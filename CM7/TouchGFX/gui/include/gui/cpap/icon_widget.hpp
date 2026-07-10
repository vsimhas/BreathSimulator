/**
 * icon_widget.hpp - Tiny custom widget that paints a monochrome 16xN bitmap
 * mask in a single colour, on top of whatever is already on screen.
 *
 * Why this exists
 * ---------------
 * The status-bar icons are small (16x16) and only redraw on state change, so
 * a custom widget that emits one fillRect per scan-line "run of set bits" is
 * the simplest path: no PNG asset pipeline, no DMA2D blit setup, no Bitmap
 * database entries. Each icon costs 32 bytes of flash for the mask.
 *
 * Mask format
 * -----------
 * One uint16_t per scan-line. Bit 15 is the leftmost pixel, bit 0 is the
 * rightmost. The widget's width is fixed at 16 px; height is whatever the
 * caller passes in to setIcon() (default 16). The caller still owns the
 * mask memory (typically a static const array in flash).
 */
#ifndef CPAP_ICON_WIDGET_HPP
#define CPAP_ICON_WIDGET_HPP

#include <touchgfx/widgets/Widget.hpp>
#include <touchgfx/hal/Types.hpp>

namespace cpap
{

class IconWidget : public touchgfx::Widget
{
public:
    IconWidget();

    /**
     * Install (or change) the bitmap mask. Pass nullptr to draw nothing.
     * Sets the widget's size to 16 x height; caller is still responsible
     * for setXY() / setVisible(true) / add() as usual.
     */
    void setIcon(const uint16_t* mask, uint8_t height = 16);

    void                setColor(touchgfx::colortype c);
    touchgfx::colortype getColor() const { return color_; }

    void    setAlpha(uint8_t a);
    uint8_t getAlpha() const { return alpha_; }

    /**
     * Visibility helper that always leaves the framebuffer correct.
     * touchgfx::Drawable::setVisible() only flips a flag and does NOT
     * invalidate, so toggling without invalidate() would leave stale
     * pixels on screen. This wrapper invalidates on either side of the
     * transition.
     */
    void setVisibleAndInvalidate(bool v);

    /* TouchGFX Widget overrides */
    virtual touchgfx::Rect getSolidRect() const
    {
        /* Mask is sparse - we don't claim any pixels as solid, so the
         * status-bar background underneath stays visible between strokes. */
        return touchgfx::Rect();
    }

    virtual void draw(const touchgfx::Rect& invalidatedArea) const;

private:
    const uint16_t*     mask_;     /* nullptr => no rendering             */
    uint8_t             height_;
    touchgfx::colortype color_;
    uint8_t             alpha_;
};

} /* namespace cpap */

#endif /* CPAP_ICON_WIDGET_HPP */
