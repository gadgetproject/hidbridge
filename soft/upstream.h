/*! \file upstream.h
 *
 *  \brief Upstream USB driver API
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

#include "hid_bridge.h"

/**
 * @brief Initialise Upstream USB driver
 * @return false on success
 */
bool upstream_init(void);

/**
 * @brief Inject a keypress into the stream
 * @param ascii character to press or \0 to release
 * @return false if keypress not queued
 */
bool upstream_emit(char ascii);

/**
 * @brief Queue a HID report
 * @param report to queue
 * @param length of report in octets
 * @return false if report could not be queued
 */
bool upstream_report_queue(const uint8_t* report, size_t length);

/**
 * @brief Register a callback for incoming HID reports
 * @param callback on report
 * @return previous callback or NULL if none
 */
bridge_report upstream_report_callback(bridge_report callback);
