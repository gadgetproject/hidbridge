/*! \file prompt.c
 *
 *  \brief User prompt driver
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

#include <stdarg.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(prompt);

static struct
{
    struct k_work_delayable work;
    struct k_mutex mutex;
    unsigned backspace;     /**< number of backspaces to emit */
    unsigned length;        /**< number of text[] characters emitted */
    bool down;              /**< true when key down last emitted */
    char text[80];          /**< text prompt, NIL terminated */
} prompt;

/** Period between up and down keypresses */
#define PROMPT_KEY_PERIOD K_MSEC(10)

/**
 * @brief State machine for 'typing' prompt
 * @param [in] self work object
 */
static void prompt_worker(struct k_work* self)
{
    k_mutex_lock(&prompt.mutex, K_FOREVER);

    /* Release key? */
    if (prompt.down)
    {
        if (upstream_emit('\0'))
        {
            prompt.down = false;
        }
    }
    /* Delete previous keypress? */
    else if (prompt.backspace)
    {
        if (upstream_emit('\b'))
        {
            prompt.down = true;
            prompt.backspace--;
        }
    }
    /* Emit new keypress? */
    else if (prompt.text[prompt.length])
    {
        if (upstream_emit(prompt.text[prompt.length]))
        {
            prompt.down = true;
            prompt.length++;
            __ASSERT(prompt.length < sizeof(prompt.text), "prompt.text overrun");
        }
    }
    else
    {
        /* All done; don't schedule another call */
        goto done;
    }
    k_work_schedule(k_work_delayable_from_work(self), PROMPT_KEY_PERIOD);
done:
    k_mutex_unlock(&prompt.mutex);
}

bool prompt_init(void)
{
    /* We use a delayable worker to emit HID reports and 'type' the prompt */
    k_work_init_delayable(&prompt.work, prompt_worker);
    return true;
}

void prompt_message(const char* msg, ...)
{
    k_mutex_lock(&prompt.mutex, K_FOREVER);

    /* Erase any previous prompt */
    prompt.backspace += prompt.length;
    prompt.length = 0;

    int written;
    if (!msg)
    {
        /* NULL message is equivalent to "" */
        written = 0;
    }
    else
    {
        /* Render msg in printf-style */
        va_list args;
        va_start (args, msg);
        written = vsnprintf(prompt.text, sizeof(prompt.text), msg, args);
        va_end (args);
    }

    if (written >= (int)sizeof(prompt.text))
    {
        /* Clip message */
        written = sizeof(prompt.text)-1;
    }

    /* Terminate message */
    prompt.text[written] = '\0';
    LOG_INF("Prompt: delete %u emit '%s'/%u", prompt.backspace, prompt.text, prompt.length);

    /* Start typing */
    k_work_schedule(&prompt.work, PROMPT_KEY_PERIOD);

    k_mutex_unlock(&prompt.mutex);
}

static int cmd_prompt(const struct shell *sh, size_t argc, char **argv)
{
    prompt_message(argc > 1 ? argv[1] : NULL);
    return 0;
}

SHELL_CMD_ARG_REGISTER(prompt, NULL, "Display user prompt", cmd_prompt, 0, 1);
