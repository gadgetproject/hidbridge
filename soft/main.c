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

#include "prompt.h"
#include "upstream.h"

#include <zephyr/kernel.h>

static void test_timer_handler(struct k_timer *dummy)
{
    ARG_UNUSED(dummy);

    static int count;
    if (count & 1)
    {
        prompt_message("Now you don't");
    }
    else
    {
        prompt_message("Now you see me #%d", 1+count/2);
    }
    count++;
}
static K_TIMER_DEFINE(test_timer, test_timer_handler, NULL);

int main(void)
{
    bool ok = true;

    ok = ok && prompt_init();
    ok = ok && upstream_init();

    if (ok)
    {
        /* Type some test prompts */
        k_timer_start(&test_timer, K_SECONDS(2), K_SECONDS(2));
    }
    return 0;
}

