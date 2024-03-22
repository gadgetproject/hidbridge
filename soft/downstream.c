/*! \file downstream.c
 *
 *  \brief Downstream BLE driver
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

#include "downstream.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(downstream);

static downstream_device downstream_scanner;

bool downstream_init(void)
{
    int err = bt_enable(NULL);
    if (err)
    {
        LOG_ERR("bt_enable %d", err);
    }
    return !err;
}

static void downstream_found(const bt_addr_le_t *addr, int8_t rssi,
                             uint8_t type, struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));

    /* We're only interested in connectable events */
    if (type != BT_GAP_ADV_TYPE_ADV_IND)
    {
        LOG_DBG("downstream_found '%s' rejected %d", addr_str, type);
        return;
    }

    /* Parse the advertisement */
    struct extract_info
    {
        bool is_hid;
        char name[31];
    } info;
    bool extract_info(struct bt_data *data, void *user_data)
    {
        struct extract_info *info = user_data;
        size_t length = data->data_len;

        switch (data->type)
        {
        case BT_DATA_UUID16_SOME:
        case BT_DATA_UUID16_ALL:
            for (unsigned i=0; i+1 < data->data_len; i+=2)
            {
                if (sys_get_le16(&data->data[i]) == BT_UUID_HIDS_VAL)
                {
                    info->is_hid = true;
                }
            }
            break;
        case BT_DATA_NAME_SHORTENED:
        case BT_DATA_NAME_COMPLETE:
            /* Unlikely name will be this long */
            if (length >= sizeof(info->name))
                length = sizeof(info->name)-1;
            /* Copy name and terminate */
            (void)memcpy(info->name, data->data, length);
            info->name[length] = '\0';
            break;
        default:
            break;
        }
        /* Continue search */
        return true;
    }
    info.is_hid = false;
    info.name[0] = '\0';
    /* Note this call destroys the buffer */
    (void)bt_data_parse(ad, extract_info, &info);
    if (!info.is_hid)
    {
        LOG_DBG("downstrean_found '%s' filtered", addr_str);
        return;
    }

    /* Inform upper layer */
    downstream_device scanner = downstream_scanner;
    if (scanner)
    {
        LOG_DBG("downstream_found '%s'", addr_str);
        scanner(addr, info.name[0] ? info.name : NULL, rssi);
    }
    else
    {
        LOG_WRN("downstream_found '%s' discarded", addr_str);
    }
}

downstream_device downstream_scan(downstream_device callback)
{
    downstream_device previous_scanner = downstream_scanner;

    if (callback)
    {
        downstream_scanner = callback;
        /* Start scanning? */
        if (!previous_scanner)
        {
            int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, downstream_found);
            if (err) {
                LOG_ERR("bt_le_scan_start %d", err);
            }
        }
    }
    else
    {
        /* Stop scanning? */
        if (previous_scanner)
        {
            int err = bt_le_scan_stop();
            if (err) {
                LOG_ERR("bt_le_scan_stop %d", err);
            }
        }
        downstream_scanner = NULL;
    }
    return previous_scanner;
}

static int cmd_scan(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    static const struct shell *the_shell;


    the_shell = sh;
    void found(const bt_addr_le_t *address, const char* name, int rssi)
    {
        char address_string[BT_ADDR_LE_STR_LEN];
        bt_addr_le_to_str(address, address_string, sizeof(address_string));
        shell_print(the_shell, "SCAN: '%s' %s %d", name, address_string, rssi);
    }
    downstream_device previous = downstream_scan(found);
    k_sleep(K_SECONDS(10));
    downstream_scan(previous);

    return 0;
}

SHELL_CMD_ARG_REGISTER(scan, NULL, "Scan for devices", cmd_scan, 0, 0);
