/*
 * Copyright (C) 2011 Hans Petter Selasky. All rights reserved.
 * Copyright (C) 2022 Neel Chauhan. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb.h>

#include <memory>

#include "usb.h"

struct usb_handle {
	libusb_device_handle *handle;
	libusb_device *dev;
	unsigned char ep_in;
	unsigned char ep_out;
	unsigned char iface;
	size_t max_packet_size;
};

struct usb_ifc_info {
        /* from device descriptor */
    unsigned short dev_vendor;
    unsigned short dev_product;

    unsigned char dev_class;
    unsigned char dev_subclass;
    unsigned char dev_protocol;

    unsigned char ifc_class;
    unsigned char ifc_subclass;
    unsigned char ifc_protocol;

    unsigned char has_bulk_in;
    unsigned char has_bulk_out;

    unsigned char writable;

    char serial_number[256];
    char device_path[256];

    char interface[256];
};

typedef int (*ifc_match_func)(usb_ifc_info *ifc);

static auto& g_usb_handles_mutex = *new std::mutex();
static auto& g_usb_handles = *new std::list<usb_handle*>();

static int 
probe(std::unique_ptr<usb_handle> &h)
{
	usb_ifc_info info;
	libusb_device_descriptor ddesc;
	libusb_config_descriptor *pcfg;
	int i, j;

	if (libusb_open(h->dev, &h->handle) < 0)
		return (-1);

	if (libusb_get_device_descriptor(h->dev, &ddesc) < 0) {
		libusb_close(h->handle);
		return (-1);
	}
	memset(&info, 0, sizeof(info));

	info.dev_vendor = ddesc.idVendor;
	info.dev_product = ddesc.idProduct;
	info.dev_class = ddesc.bDeviceClass;
	info.dev_subclass = ddesc.bDeviceSubClass;
	info.dev_protocol = ddesc.bDeviceProtocol;
	info.writable = 1;

	snprintf(info.device_path, sizeof(info.device_path), "usb:%d:%d",
		 libusb_get_bus_number(h->dev), libusb_get_device_address(h->dev));

	if (ddesc.iSerialNumber != 0) {
		libusb_get_string_descriptor_ascii(h->handle, ddesc.iSerialNumber,
		    (unsigned char *)info.serial_number, sizeof(info.serial_number));
	}
	if (libusb_get_active_config_descriptor(h->dev, &pcfg)) {
		libusb_close(h->handle);
		return (-1);
	}

	for (i = 0; i < pcfg->bNumInterfaces; i++) {

		h->ep_in = 0;
		h->ep_out = 0;
		h->iface = i;

		for (j = 0; j < pcfg->interface[i].altsetting[0].bNumEndpoints; j++) {

			unsigned char temp = pcfg->interface[i].altsetting[0].
			endpoint[j].bEndpointAddress;
			unsigned char type = pcfg->interface[i].altsetting[0].
			endpoint[j].bmAttributes & 0x03;

			/* check for BULK endpoint */
			if ((type & 0x03) == 0x02) {
				/* check for IN endpoint */
				if (temp & 0x80)
					h->ep_in = temp;
				else
					h->ep_out = temp;
			}
		}

		info.ifc_class = pcfg->interface[i].altsetting[0].bInterfaceClass;
		info.ifc_subclass = pcfg->interface[i].altsetting[0].bInterfaceSubClass;
		info.ifc_protocol = pcfg->interface[i].altsetting[0].bInterfaceProtocol;
		info.has_bulk_in = (h->ep_in != 0);
		info.has_bulk_out = (h->ep_out != 0);

		h->max_packet_size = libusb_get_max_iso_packet_size(h->dev, h->ep_in);

		if (libusb_claim_interface(h->handle, h->iface) < 0)
			continue;

		if (is_adb_interface(info.ifc_class, info.ifc_subclass, info.ifc_protocol)) {
			std::lock_guard<std::mutex> lock(g_usb_handles_mutex);
			g_usb_handles.push_back(h);
			register_usb_transport(h, info.serial_number, dev_path, h->writeable);
			libusb_free_config_descriptor(pcfg);
			return (0);
		}
		libusb_release_interface(h->handle, h->iface);
	}

	libusb_free_config_descriptor(pcfg);
	libusb_close(h->handle);
	return (-1);
}

static std::unique_ptr<usb_handle>
enumerate()
{
	static libusb_context *ctx = NULL;
	std::unique_ptr<usb_handle> h;
	libusb_device **ppdev;
	ssize_t ndev;
	ssize_t x;
	bool done = false;

	if (ctx == NULL)
		libusb_init(&ctx);

	ndev = libusb_get_device_list(ctx, &ppdev);
	for (x = 0; x < ndev; x++) {

		h.reset(new usb_handle);

		h->dev = ppdev[x];

		if (probe(h) == 0) {
			libusb_ref_device(h->dev);
			libusb_free_device_list(ppdev, 1);
			return (h);
		}
	}
	h.reset();
	libusb_free_device_list(ppdev, 1);
	return (nullptr);
}

int
usb_write(usb_handle* h, const void *_data, int len)
{
	int actlen;

	if (libusb_bulk_transfer(h->handle, h->ep_out,
				 (unsigned char *)_data, len, &actlen, 10000) < 0)
		return (-1);
	return (actlen);
}

int
usb_read(usb_handle *h, void *_data, int len)
{
	int actlen;

	if (libusb_bulk_transfer(h->handle, h->ep_in,
				 (unsigned char *)_data, len, &actlen, 10000) < 0)
		return (-1);
	return (actlen);
}

int
usb_close(usb_handle* h)
{
	std::lock_guard<std::mutex> lock(g_usb_handles_mutex);
	g_usb_handles.remove(h);

	libusb_close(h->handle);
	h->handle = NULL;
	libusb_unref_device(h->dev);
	usb_reset(h);
	return (0);
}

void usb_kick(usb_handle* h)
{
        usb_close(h);
}

void
usb_reset(usb_handle* h)
{
	libusb_reset_device(h->handle);
}

size_t
usb_get_max_packet_size(usb_handle* h)
{
    return h->max_packet_size;
}

static void
kick_disconnected_devices()
{
    std::lock_guard<std::mutex> lock(g_usb_handles_mutex);
    // kick any devices in the device list that were not found in the device scan
    for (usb_handle* usb : g_usb_handles) {
        if (!usb->mark) {
            usb_kick(usb);
        } else {
            usb->mark = false;
        }
    }
}

static void
device_poll_thread()
{
    adb_thread_setname("device poll");
    D("Created device thread");
    while (true) {
        //find_usb_device("/dev/bus/usb", register_device);
        enumerate();
        adb_notify_device_scan_complete();
        kick_disconnected_devices();
        std::this_thread::sleep_for(1s);
    }
}

void
usb_init()
{
    struct sigaction actions;
    memset(&actions, 0, sizeof(actions));
    sigemptyset(&actions.sa_mask);
    actions.sa_flags = 0;
    actions.sa_handler = [](int) {};
    sigaction(SIGALRM, &actions, nullptr);

    std::thread(device_poll_thread).detach();
}


void
usb_cleanup() {}
