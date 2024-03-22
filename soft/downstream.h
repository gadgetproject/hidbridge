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
 */
downstream_device downstream_scan(downstream_device callback);
