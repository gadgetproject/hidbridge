/*! \file main.c
 *
 *  \brief Entry point
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

#include <zephyr/kernel.h>

extern int upstream_init(void);
extern int upstream_emit(char c);


static void test_emit_worker(struct k_work *work)
{
    ARG_UNUSED(work);

    static const char msg[] = "Hello, World!\n";
    static uint8_t index;

    /* Press key */
    int err = upstream_emit(msg[index]);
    if (err)
    {
        printk("test emit '%c': %d\n", msg[index], err);
    }
    else
    {
        /* Release key */
        for(;;)
        {
            err = upstream_emit(0);
            if (!err)
            {
                /* Success */
                break;
            }
            else if (err != -EBUSY)
            {
                /* Failure */
                printk("test emit release: %d\n", err);
                break;
            }

            /* Retry */
            k_sleep(K_MSEC(10));
        }
    }
    if (!msg[index++])
    {
        /* Wrap back to start of message */
        index = 0;
    }
}
static struct k_work test_emit;

static void test_timer_handler(struct k_timer *dummy)
{
    ARG_UNUSED(dummy);

	k_work_submit(&test_emit);
}
static K_TIMER_DEFINE(test_timer, test_timer_handler, NULL);

int main(void)
{
    int err;

    err = upstream_init();
    if (err)
    {
        printk("Upstream init failed: %d\n", err);
        return err;
    }

    /* Type a recurring string */
	k_work_init(&test_emit, test_emit_worker);
	k_timer_start(&test_timer, K_SECONDS(2), K_SECONDS(2));
    return 0;
}

