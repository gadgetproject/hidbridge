/*! \file downstream.h
 *
 *  \brief Downstream BLE driver API
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

#include <zephyr/bluetooth/addr.h>

/**
 * @brief Initialise Downstream BLE driver
 * @return false on failure
 */
bool downstream_init(void);

/**
 * @brief Callback for a scanned device
 * @param [in] address of device scanned
 * @param [in] name of device scanned or NULL if no name
 * @param [in] rssi of scan response
 */
typedef void (*downstream_device)(const bt_addr_le_t *address,
                                  const char* name, int rssi);

/**
 * @brief Start/Stop scanning for connectable devices
 * @param callback on scan or NULL to stop scanning
 * @return previous callback or NULL if none
 * @note this function returns immediately and callback() is called from another thread
 */
downstream_device downstream_scan(downstream_device callback);

/**
 * @brief Callback for a connected device
 */
typedef void (*downstream_connected)(void);

/**
 * @brief Connect to device
 * @param address of scanned device or NULL to use bonded device
 * @param callback called when connected or NULL to disconnect
 * @return false if connection fails otherwise callback() called before returning true
 */
bool downstream_connect(const bt_addr_le_t *address, downstream_connected callback);
