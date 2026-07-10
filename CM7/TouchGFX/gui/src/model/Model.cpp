#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>

#include <gui/cpap/cpap_menu.hpp>

extern "C" {
    #include "rotary_input.h"
    #include "dbg_log.h"
}

/**
 * Model::tick() is invoked once per frame by FrontendApplication::handleTickEvent(),
 * regardless of which screen is active or whether any widget is registered as a
 * timer widget. We use it as the always-on pump for the rotary input so the
 * menu state machine advances even if a screen forgets to register itself.
 *
 * The view also polls the rotary in its own handleTickEvent() (when registered),
 * but RotaryInput_Pop*() drains the latches so processing the events twice is
 * harmless - whichever path runs first wins.
 */
Model::Model() : modelListener(0)
{

}

void Model::tick()
{
    /* Heartbeat at DEBUG level - left in place for diagnostics but does not
     * show in the default VCP output. */
    static uint32_t s_tick_count = 0U;
    if ((++s_tick_count % 600U) == 0U)
    {
        DBG_D("UI", "Model::tick alive n=%lu", (unsigned long)s_tick_count);
    }

    int8_t d = RotaryInput_PopDelta();
    if (d != 0)
    {
        DBG_D("UI", "rotary delta=%d page=%u mode=%u sel=%u",
              (int)d,
              (unsigned)cpap::getMenu().page(),
              (unsigned)cpap::getMenu().mode(),
              (unsigned)cpap::getMenu().selectedIndex());
        cpap::getMenu().onRotaryDelta(static_cast<int>(d));
    }

    /* Process at most ONE press per tick. The driver debounces bursts at
     * the source, but if two genuine presses come in close enough that
     * they're popped together, we still want the user to see the
     * intermediate page change before the second one applies. The
     * remaining presses stay latched and fire on the next tick. */
    uint8_t p = RotaryInput_PopPress();
    if (p > 0U)
    {
        cpap::getMenu().onButtonPress();
        DBG_D("UI", "press -> page=%u mode=%u sel=%u",
              (unsigned)cpap::getMenu().page(),
              (unsigned)cpap::getMenu().mode(),
              (unsigned)cpap::getMenu().selectedIndex());

        /* Return any extra presses to the latch for the next frame. */
        if (p > 1U)
        {
            RotaryInput_PushBackPresses((uint8_t)(p - 1U));
        }
    }
}
