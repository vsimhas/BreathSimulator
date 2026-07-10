/**
 * icon_widget.cpp - mask-driven custom widget. See icon_widget.hpp for the
 * mask format. The render loop converts each row's set bits into runs and
 * forwards them to HAL::lcd().fillRect() in absolute screen coordinates,
 * matching how touchgfx::Box draws.
 */
#include <gui/cpap/icon_widget.hpp>

#include <touchgfx/hal/HAL.hpp>
#include <touchgfx/lcd/LCD.hpp>

namespace cpap
{

IconWidget::IconWidget()
    : mask_(nullptr), height_(16U), color_(0xFFFFu), alpha_(255U)
{
    setWidthHeight(16, 16);
}

void IconWidget::setIcon(const uint16_t* mask, uint8_t height)
{
    mask_   = mask;
    height_ = (height > 0U) ? height : 16U;
    setWidthHeight(16, height_);
    invalidate();
}

void IconWidget::setColor(touchgfx::colortype c)
{
    if (c != color_)
    {
        color_ = c;
        invalidate();
    }
}

void IconWidget::setAlpha(uint8_t a)
{
    if (a != alpha_)
    {
        alpha_ = a;
        invalidate();
    }
}

void IconWidget::setVisibleAndInvalidate(bool v)
{
    if (isVisible() == v)
    {
        return;
    }
    if (isVisible())
    {
        /* Going visible -> invisible. Mark old footprint dirty so the
         * status bar background repaints over it. */
        invalidate();
    }
    setVisible(v);
    if (isVisible())
    {
        /* Going invisible -> visible. Now request the new pixels. */
        invalidate();
    }
}

void IconWidget::draw(const touchgfx::Rect& invalidatedArea) const
{
    if (mask_ == nullptr || alpha_ == 0U)
    {
        return;
    }

    const int16_t a_x0 = invalidatedArea.x;
    const int16_t a_y0 = invalidatedArea.y;
    const int16_t a_x1 = (int16_t)(invalidatedArea.x + invalidatedArea.width);
    const int16_t a_y1 = (int16_t)(invalidatedArea.y + invalidatedArea.height);

    for (uint8_t y = 0U; y < height_; ++y)
    {
        if ((int16_t)y < a_y0 || (int16_t)y >= a_y1)
        {
            continue;
        }
        uint16_t bits = mask_[y];
        if (bits == 0u)
        {
            continue;
        }

        /* Walk the row, skipping clear bits, emitting one fillRect per
         * run of set bits. 16 columns max, so the inner loop is tiny. */
        uint8_t x = 0U;
        while (x < 16U)
        {
            while (x < 16U && (bits & (uint16_t)(1u << (15U - x))) == 0u)
            {
                ++x;
            }
            if (x >= 16U)
            {
                break;
            }
            const uint8_t run_start = x;
            while (x < 16U && (bits & (uint16_t)(1u << (15U - x))) != 0u)
            {
                ++x;
            }
            /* Run is [run_start, x-1]. Clip to invalidatedArea and emit. */
            int16_t rx = (int16_t)run_start;
            int16_t rw = (int16_t)(x - run_start);
            if (rx < a_x0)
            {
                rw -= (int16_t)(a_x0 - rx);
                rx = a_x0;
            }
            if (rx + rw > a_x1)
            {
                rw = (int16_t)(a_x1 - rx);
            }
            if (rw <= 0)
            {
                continue;
            }

            touchgfx::Rect run(rx, (int16_t)y, rw, 1);
            translateRectToAbsolute(run);
            touchgfx::HAL::lcd().fillRect(run, color_, alpha_);
        }
    }
}

} /* namespace cpap */
