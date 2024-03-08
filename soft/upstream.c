/*! \file upstream.c
 *
 *  \brief Upstream USB driver
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "upstream.h"

#include <zephyr/kernel.h>
#include <zephyr/init.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

static bool upstream_configured;
static const struct device *upstream_hid0;
static ATOMIC_DEFINE(upstream_flags, 1);

bool upstream_emit(char ascii)
{
    int err, wrote;

    if (!upstream_configured || atomic_test_and_set_bit(upstream_flags, 0))
    {
        /* busy */
        return false;
    }

    /* Trivial US keyboard */
    static const uint8_t keymap[127] =
    {
        ['\0'] = 0x00,      /* Key release */

        /* Unshifted */     /* Shifted */
        ['a'] = 0x04,       ['A'] = 0x84,
        ['b'] = 0x05,       ['B'] = 0x85,
        ['c'] = 0x06,       ['C'] = 0x86,
        ['d'] = 0x07,       ['D'] = 0x87,
        ['e'] = 0x08,       ['E'] = 0x88,
        ['f'] = 0x09,       ['F'] = 0x89,
        ['g'] = 0x0a,       ['G'] = 0x8a,
        ['h'] = 0x0b,       ['H'] = 0x8b,
        ['i'] = 0x0c,       ['I'] = 0x8c,
        ['j'] = 0x0d,       ['J'] = 0x8d,
        ['k'] = 0x0e,       ['K'] = 0x8e,
        ['l'] = 0x0f,       ['L'] = 0x8f,
        ['m'] = 0x10,       ['M'] = 0x90,
        ['n'] = 0x11,       ['N'] = 0x91,
        ['o'] = 0x12,       ['O'] = 0x92,
        ['p'] = 0x13,       ['P'] = 0x93,
        ['q'] = 0x14,       ['Q'] = 0x94,
        ['r'] = 0x15,       ['R'] = 0x95,
        ['s'] = 0x16,       ['S'] = 0x96,
        ['t'] = 0x17,       ['T'] = 0x97,
        ['u'] = 0x18,       ['U'] = 0x98,
        ['v'] = 0x19,       ['V'] = 0x99,
        ['w'] = 0x1a,       ['W'] = 0x9a,
        ['x'] = 0x1b,       ['X'] = 0x9b,
        ['y'] = 0x1c,       ['Y'] = 0x9c,
        ['z'] = 0x1d,       ['Z'] = 0x9d,

        ['1'] = 0x1e,       ['!'] = 0x9e,
        ['2'] = 0x1f,       ['@'] = 0x9f,
        ['3'] = 0x20,       ['#'] = 0xa0,
        ['4'] = 0x21,       ['$'] = 0xa1,
        ['5'] = 0x22,       ['%'] = 0xa2,
        ['6'] = 0x23,       ['^'] = 0xa3,
        ['7'] = 0x24,       ['&'] = 0xa4,
        ['8'] = 0x25,       ['*'] = 0xa5,
        ['9'] = 0x26,       ['('] = 0xa6,
        ['0'] = 0x27,       [')'] = 0xa7,

        ['\n'] = 0x28,
        [' '] = 0x2c,

        ['-'] = 0x2d,       ['_'] = 0xad,
        ['='] = 0x2e,       ['+'] = 0xae,
        ['['] = 0x2f,       ['{'] = 0xaf,
        [']'] = 0x30,       ['}'] = 0xb0,
        ['\\'] = 0x31,      ['|'] = 0xb1,
        [';'] = 0x33,       [':'] = 0xb3,
        ['\''] = 0x34,      ['"'] = 0xb4,
        ['`'] = 0x35,       ['~'] = 0xb5,
        [','] = 0x36,       ['<'] = 0xb6,
        ['.'] = 0x37,       ['>'] = 0xb7,
        ['/'] = 0x38,       ['?'] = 0xb8,
    };

    /* ASCII code available? */
    if (ascii < 0 || (ascii && !keymap[(unsigned)ascii]))
    {
        return -EINVAL;
    }

    /* Generate key press */
    uint8_t report[8] = {0};
    report[0] = (keymap[(unsigned)ascii] & 0x80) ? HID_KBD_MODIFIER_LEFT_SHIFT : 0;
    report[7] = keymap[(unsigned)ascii] & 0x7f;

    err = hid_int_ep_write(upstream_hid0, report, sizeof(report), &wrote);
    if (err)
    {
        (void)atomic_test_and_clear_bit(upstream_flags, 0);
        printk("upstream write failed: %d\n", err);
        return false;
    }
    if (wrote != sizeof(report))
    {
        printk("upstream wrote %u/%u\n", wrote, sizeof(report));
    }
    return true;
}

static void upstream_int_in_ready_cb(const struct device *dev)
{
    ARG_UNUSED(dev);
    if (!atomic_test_and_clear_bit(upstream_flags, 0))
    {
        printk("upstream unexpected ready callback\n");
    }
}

static void upstream_protocol_cb(const struct device *dev, uint8_t protocol)
{
    printk("upstream protocol %u\n", protocol);
}

static void upstream_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch (status)
    {
    case USB_DC_RESET:
        printk("upstream USB DC reset\n");
        upstream_configured = false;
        (void)atomic_test_and_clear_bit(upstream_flags, 0);
        break;
    case USB_DC_CONFIGURED:
        printk("upstream USB DC configured\n");
        upstream_configured = true;
        break;
    case USB_DC_SOF:
        break;
    default:
        printk("upstream USB DC status %u\n", status);
        break;
    }
}

bool upstream_init(void)
{
    int err;

    upstream_hid0 = device_get_binding("HID_0");
    if (!upstream_hid0)
    {
        printk("Cannot get USB HID Device\n");
        return false;
    }

    static const uint8_t desc[] = HID_KEYBOARD_REPORT_DESC();
    static const struct hid_ops ops =
    {
        .int_in_ready = upstream_int_in_ready_cb,
        .protocol_change = upstream_protocol_cb,
    };

    usb_hid_register_device(upstream_hid0, desc, sizeof(desc), &ops);

    atomic_set_bit(upstream_flags, 0);

    err = usb_hid_set_proto_code(upstream_hid0, HID_BOOT_IFACE_CODE_NONE);
    if (err)
    {
        printk("Failed to set Protocol Code: %d\n", err);
        return false;
    }

    err = usb_hid_init(upstream_hid0);
    if (err)
    {
        printk("Failed to initialise USB: %d\n", err);
        return false;
    }

    err = usb_enable(upstream_status_cb);
    if (err)
    {
        printk("Failed to enable USB: %d\n", err);
        return false;
    }

    return true;
}
