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
#include "prompt.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>    // LOG_PROCESS()

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

/**
 * @brief Callback from BLE stack when a device is scanned
 */
static void downstream_scanned_cb(const bt_addr_le_t *address, int8_t rssi,
                                  uint8_t type, struct net_buf_simple *ad)
{
    /* For debug logging */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(address, addr_str, sizeof(addr_str));

    /* We're only interested in connectable events */
    if (type != BT_GAP_ADV_TYPE_ADV_IND)
    {
        LOG_DBG("downstream_scanned_cb '%s' rejected %d", addr_str, type);
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
        LOG_DBG("downstream_scanned_cb '%s'", addr_str);
        scanner(address, info.name[0] ? info.name : NULL, rssi);
    }
    else
    {
        LOG_WRN("downstream_scanned_cb '%s' discarded", addr_str);
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
            int err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, downstream_scanned_cb);
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

/**
 * @brief Events signal BLE state changes
 */
K_EVENT_DEFINE(downstream_events);
enum
{
    DOWNSTREAM_EVENT_CONNECTED = 1u<<0,
    DOWNSTREAM_EVENT_DISCONNECTED = 1u<<1,
    DOWNSTREAM_EVENT_ENCRYPTED = 1u<<2,
    DOWNSTREAM_EVENT_DISCOVERED = 1u<<3,
};

/**
 * @brief Wait for one of a set of events to occur
 * @param [in] events bitmap of events to wait on
 * @return bitmap of signalled events
 */
static uint32_t downstream_wait(uint32_t events)
{
    const unsigned timeout_ms = 30000;
    const unsigned poll_ms = 100;
    for(unsigned ms = 0; ms < timeout_ms; ms += poll_ms)
    {
        /* Flush the log */
        while(LOG_PROCESS());

        /* Wait a bit for an event, adding disconnected */
        uint32_t signalled = k_event_wait(&downstream_events,
                                          events | DOWNSTREAM_EVENT_DISCONNECTED,
                                          false, K_MSEC(poll_ms));

        /* Anything happened? */
        if (signalled)
        {
            return signalled & events;
        }
    }

    /* Timeout */
    LOG_WRN("downstream_wait() timeout");
    return 0;
}

/**
 * @brief The current BLE connection or NULL if disconnected
 */
static struct bt_conn *downstream_conn;

/**
 * @brief Callback from BLE stack when a device connection attempt succeeds or fails
 */
static void downstream_connected_cb(struct bt_conn *conn, uint8_t conn_err)
{
    /* For debug logging */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (conn_err)
    {
        bt_conn_unref(downstream_conn);
        downstream_conn = NULL;
        LOG_WRN("%s connect failed %d", addr_str, conn_err);
        (void)k_event_set(&downstream_events, DOWNSTREAM_EVENT_DISCONNECTED);
        return;
    }

    LOG_INF("%s connected", addr_str);
    (void)k_event_set(&downstream_events, DOWNSTREAM_EVENT_CONNECTED);
}

static void downstream_passkey_display_cb(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey advised for %s: %06u", addr, passkey);

    /* Display passkey prompt */
    prompt_message(PROMPT_PASSKEY, passkey);
}
static void downstream_passkey_confirm_cb(struct bt_conn *conn, unsigned int passkey)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Passkey accepted for %s: %06u", addr, passkey);

    /* Display passkey prompt */
    prompt_message(PROMPT_PASSKEY, passkey);

    bt_conn_auth_passkey_confirm(conn);
}
static void downstream_pairing_cancel_cb(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_WRN("Pairing cancelled: %s", addr);

    /* Clear passkey prompt */
    prompt_message(NULL);
}
static void downstream_pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Pairing completed: %s, bonded: %d", addr, bonded);

    /* Clear passkey prompt */
    prompt_message(NULL);
}
static void downstream_pairing_failed_cb(struct bt_conn *conn, enum bt_security_err reason)
{
    char addr[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_WRN("Pairing failed conn: %s, reason %d", addr, reason);

    /* Clear passkey prompt */
    prompt_message(NULL);
}

static struct bt_conn_auth_cb downstream_passkey_callbacks = {
    .passkey_display = downstream_passkey_display_cb,
    .passkey_confirm = downstream_passkey_confirm_cb,
    .cancel = downstream_pairing_cancel_cb,
};
static struct bt_conn_auth_info_cb downstream_pairing_callbacks = {
    .pairing_complete = downstream_pairing_complete_cb,
    .pairing_failed = downstream_pairing_failed_cb
};

static void downstream_encrypted_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
    /* For debug logging */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    if (err)
    {
        LOG_ERR("%s failed to encrypt level %d err %d", addr_str, level, err);
        (void)bt_conn_disconnect(conn, err);
        return;
    }
    LOG_INF("%s encrypted level %d", addr_str, level);

    (void)k_event_set(&downstream_events, DOWNSTREAM_EVENT_ENCRYPTED);
}

/**
 * @brief Callback from BLE stack when a device connection is dropped
 */
static void downstream_disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    /* For debug logging */
    char addr_str[BT_ADDR_LE_STR_LEN];
    bt_addr_le_to_str(bt_conn_get_dst(conn), addr_str, sizeof(addr_str));

    bt_conn_unref(downstream_conn);
    downstream_conn = NULL;
    LOG_INF("%s disconnected %d", addr_str, reason);
    (void)k_event_set(&downstream_events, DOWNSTREAM_EVENT_DISCONNECTED);
}

BT_CONN_CB_DEFINE(downstream_conn_callbacks) = {
    .connected = downstream_connected_cb,
    .disconnected = downstream_disconnected_cb,
    .security_changed = downstream_encrypted_cb,
};

/**
 * @brief Handles discovered
 */
static struct {
    struct
    {
        uint16_t handle;
        struct bt_gatt_subscribe_params sub_params;
        struct bt_gatt_discover_params sub_disc_params;
    } boot_in;
    struct
    {
        uint16_t handle;
        struct bt_gatt_subscribe_params sub_params;
        struct bt_gatt_discover_params sub_disc_params;
    } report[10];
} downstream_handles;

/**
 * @brief bt_gatt_subscribe() callback
 * @param conn
 * @param params
 * @param data
 * @param length
 * @return
 */
static uint8_t downstream_notify_cb(struct bt_conn *conn,
                                    struct bt_gatt_subscribe_params *params,
                                    const void *data, uint16_t length)
{
    const char* value = NULL;
    if (params->value_handle == downstream_handles.boot_in.handle)
    {
        value = "BOOT_IN";
        /* TODO - handle keypress */
    }
    else for (unsigned i = 0; i < ARRAY_SIZE(downstream_handles.report); i++)
    {
        if (params->value_handle == downstream_handles.report[i].handle)
        {
            value = "REPORT";
            /* TODO - handle keypress */
        }
    }
    if (!value)
    {
        LOG_WRN("unexpected handle 0x%04X", params->value_handle);
        value = "?unexpected?";
    }

    LOG_HEXDUMP_INF(data, length, value);

    return BT_GATT_ITER_CONTINUE;
}

/**
 * @brief bt_gatt_discover() callback
 * @param conn
 * @param attr
 * @param params
 * @return
 */
static uint8_t downstream_discover_cb(struct bt_conn *conn,
                                      const struct bt_gatt_attr *attr,
                                      struct bt_gatt_discover_params *params)
{
    /* Discovered something? */
    if (attr)
    {
        LOG_INF("downstream attribute type %d handle 0x%04X", params->type, attr->handle);

        if (params->type == BT_GATT_DISCOVER_CHARACTERISTIC) {
            struct bt_gatt_chrc *chrc = attr->user_data;
            struct bt_gatt_subscribe_params *sub_params = NULL;

            char uuid_str[37];
            bt_uuid_to_str(chrc->uuid, uuid_str, sizeof(uuid_str));

            if (bt_uuid_cmp(chrc->uuid, BT_UUID_HIDS_REPORT) == 0)
            {
                LOG_INF("%s: REPORT", uuid_str);
                for (unsigned i = 0; i < ARRAY_SIZE(downstream_handles.report); i++)
                {
                    if (!downstream_handles.report[i].handle)
                    {
                        downstream_handles.report[i].handle = chrc->value_handle;
                        sub_params = &downstream_handles.report[i].sub_params;
                        sub_params->disc_params = &downstream_handles.report[i].sub_disc_params;
                        break;
                    }
                }
                if (!sub_params)
                {
                    LOG_WRN("report slots exhausted");
                }
            }
            else if (bt_uuid_cmp(chrc->uuid, BT_UUID_HIDS_BOOT_KB_IN_REPORT) == 0)
            {
                LOG_INF("%s: BOOT_IN", uuid_str);
                downstream_handles.boot_in.handle = chrc->value_handle;
                sub_params = &downstream_handles.boot_in.sub_params;
                sub_params->disc_params = &downstream_handles.boot_in.sub_disc_params;
            }
            else
            {
                LOG_INF("%s: ignored", uuid_str);
            }

            /* Subscribe? */
            if (sub_params != NULL)
            {
                sub_params->value = BT_GATT_CCC_NOTIFY;
                sub_params->value_handle = chrc->value_handle;
                sub_params->ccc_handle = 0;
                sub_params->end_handle = params->end_handle;
                sub_params->notify = downstream_notify_cb;
                int err = bt_gatt_subscribe(conn, sub_params);
                if (err)
                {
                    LOG_WRN("subscribe 0x%04X %d", chrc->value_handle, err);
                }
            }

            return BT_GATT_ITER_CONTINUE;
        }
    }

    /* TODO: check mandatory characteristics */

    (void)k_event_set(&downstream_events, DOWNSTREAM_EVENT_DISCOVERED);
    return BT_GATT_ITER_STOP;
}

bool downstream_connect(const bt_addr_le_t *address, downstream_connected callback)
{
    /* Disconnect? */
    if (downstream_conn)
    {
        (void)bt_conn_disconnect(downstream_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        (void)downstream_wait(DOWNSTREAM_EVENT_DISCONNECTED);
    }
    (void)k_event_set(&downstream_events, 0);

    /* Just disconnect? */
    if (!callback)
    {
        if (address)
        {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(address, addr_str, sizeof(addr_str));
            LOG_ERR("downstream_connect(%s, NULL)", addr_str);
            return false;
        }
        return true;
    }

    /* Reconnect to bonded device? */
    bt_addr_le_t bond_address;
    if (!address)
    {
        void get_bond_address(const struct bt_bond_info *info, void *user_data)
        {
            char addr_str[BT_ADDR_LE_STR_LEN];
            bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));
            LOG_INF("%s bonded", addr_str);

            (*(bt_addr_le_t *)user_data) = info->addr;
        }

        bt_addr_le_copy(&bond_address, BT_ADDR_LE_NONE);
        bt_foreach_bond(BT_ID_DEFAULT, get_bond_address, &bond_address);
        if (bt_addr_le_eq(&bond_address, BT_ADDR_LE_NONE))
        {
            LOG_ERR("downstream_connect not bonded");
            return false;
        }
        address = &bond_address;
    }
    else
    {
        /* Remove any bonds */
        (void)bt_unpair(BT_ID_DEFAULT, NULL);
    }

    /* Initiate connection */
    int err = bt_conn_le_create(address,
                                BT_CONN_LE_CREATE_CONN,
                                BT_LE_CONN_PARAM_DEFAULT,
                                &downstream_conn);
    if (err)
    {
        LOG_ERR("downstream_connect connect failed %d\n", err);
        return false;
    }

    /* Wait for connect */
    if (!downstream_wait(DOWNSTREAM_EVENT_CONNECTED))
    {
        /* Connection failed */
        return false;
    }

    /* Initiate encryption */
    (void)bt_conn_auth_cb_register(&downstream_passkey_callbacks);
    (void)bt_conn_auth_info_cb_register(&downstream_pairing_callbacks);
    err = bt_conn_set_security(downstream_conn, BT_SECURITY_L2);
    if (err && err != -EBUSY)
    {
        LOG_ERR("downstream_connect security failed %d\n", err);
        (void)bt_conn_disconnect(downstream_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    /* Wait for encryption */
    if (!downstream_wait(DOWNSTREAM_EVENT_ENCRYPTED))
    {
        /* Bonding failed */
        return false;
    }

    /* Initiate discovery */
    struct bt_gatt_discover_params discover_params =
    {
        .func = downstream_discover_cb,
        .type = BT_GATT_DISCOVER_CHARACTERISTIC,
        .start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
        .end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
    };

    err = bt_gatt_discover(downstream_conn, &discover_params);
    if (err)
    {
        LOG_ERR("downstream_connect discover failed %d\n", err);
        (void)bt_conn_disconnect(downstream_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }

    /* Wait for discovery */
    if (!downstream_wait(DOWNSTREAM_EVENT_DISCOVERED))
    {
        /* Discovery failed */
        return false;
    }

    callback();

    return true;
}

/**************************************************************************/
#ifdef CONFIG_SHELL

static int downstream_cmd_scan(const struct shell *sh, size_t argc, char **argv)
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

static int downstream_cmd_connect(const struct shell *sh, size_t argc, char **argv)
{
    static const struct shell *the_shell;

    if (argc < 2) {
        shell_error(sh, "usage: %s [public|random] UU:VV:WW:XX:YY:ZZ", argv[0]);
        return -1;
    }
    const char* type = (argc < 3) ? "public" : argv[1];
    const char* addr = (argc < 3) ? argv[1] : argv[2];

    bt_addr_le_t peer;
    int err = bt_addr_le_from_str(addr, type, &peer);
    if (err)
    {
        shell_error(sh, "Invalid peer address '%s %s' (err %d)", type, addr, err);
        return err;
    }

    the_shell = sh;
    void connected(void)
    {
        shell_print(the_shell, "CONNECTED");
    }
    if (!downstream_connect(&peer, connected))
    {
        shell_error(the_shell, "FAILED");
    }

    return 0;
}

static int downstream_cmd_disconnect(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    if (!downstream_connect(NULL, NULL))
    {
        shell_error(sh, "FAILED");
    }

    return 0;
}

static int downstream_cmd_reconnect(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    static const struct shell *the_shell;

    the_shell = sh;
    void reconnected(void)
    {
        shell_print(the_shell, "RECONNECTED");
    }
    if (!downstream_connect(NULL, reconnected))
    {
        shell_error(the_shell, "FAILED");
    }

    return 0;
}

static int downstream_cmd_bdaddr(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    bt_addr_le_t addr = {0};
    size_t one = 1;
    bt_id_get(&addr, &one);
    if (one) {
        char dev[BT_ADDR_LE_STR_LEN];
        (void)bt_addr_le_to_str(&addr, dev, BT_ADDR_LE_STR_LEN);
        shell_print(sh, "%s", dev);
    }
    return 0;
}

/* TODO: Remove when key manager exported in zephyr/bluetooth */
#include <../subsys/bluetooth/host/keys.h>

static int downstream_cmd_keys(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    void dump(struct bt_keys *key, void *data)
    {
        const struct shell* sh = data;

        char bdaddr[BT_ADDR_LE_STR_LEN];
        (void)bt_addr_le_to_str(&key->addr, bdaddr, BT_ADDR_LE_STR_LEN);
        shell_print(sh, "%d:\t%s", key->id, bdaddr);
        shell_print(sh, "\tenc_size: %d flags: 0x%x keys: 0x%x", key->enc_size, key->flags, key->keys);
        shell_print(sh, "\tLTK: %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x",
                        key->ltk.val[0], key->ltk.val[1], key->ltk.val[2], key->ltk.val[3],
                        key->ltk.val[4], key->ltk.val[5], key->ltk.val[6], key->ltk.val[7],
                        key->ltk.val[8], key->ltk.val[9], key->ltk.val[10], key->ltk.val[11],
                        key->ltk.val[12], key->ltk.val[13], key->ltk.val[14], key->ltk.val[15]);
    }

    bt_keys_foreach_type(BT_KEYS_ALL, dump, (void*)sh);

    return 0;
}

static int downstream_cmd_keys_clear(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int err = bt_unpair(BT_ID_DEFAULT, NULL);
    if (err) {
        shell_error(sh, "bt_unpair: %d", err);
    } else {
        shell_print(sh, "ok");
    }
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(downstream_keys_cmds,
    SHELL_CMD_ARG(clear, NULL,
              "Clear bluetooth keys",
              downstream_cmd_keys_clear, 0, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(downstream_cmds,
    SHELL_CMD_ARG(scan, NULL,
              "Scan for bluetooth keyboards",
              downstream_cmd_scan, 0, 0),
    SHELL_CMD_ARG(connect, NULL,
              "Connect bluetooth keyboard",
              downstream_cmd_connect, 1, 2),
    SHELL_CMD_ARG(disconnect, NULL,
              "Disconnect bluetooth keyboard",
              downstream_cmd_disconnect, 0, 0),
    SHELL_CMD_ARG(reconnect, NULL,
              "Reconnect bluetooth keyboard",
              downstream_cmd_reconnect, 0, 0),
    SHELL_CMD_ARG(bdaddr, NULL,
              "Get bluetooth address",
              downstream_cmd_bdaddr, 0, 0),
    SHELL_CMD_ARG(keys, &downstream_keys_cmds,
              "Get/Clear bluetooth keys",
              downstream_cmd_keys, 0, 0),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_ARG_REGISTER(downstream, &downstream_cmds,
               "Bluetooth client shell commands",
               NULL, 0, 0);

#endif /* CONFIG_SHELL */
