#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <lib/mouse.h>
#include <lib/getchar.h>
#include <lib/term.h>
#include <lib/misc.h>
#if defined (UEFI)
#  include <efi.h>
#endif
#if defined (BIOS)
#  include <sys/cpu.h>
#  include <sys/pic.h>
#endif

bool mouse_available = false;
size_t mouse_x = 0, mouse_y = 0;

// Sub-cell motion accumulators (in raw device units) and last-seen button state.
static long accum_x = 0, accum_y = 0;
static bool prev_left = false, prev_right = false;

// Device units that must accumulate before the pointer steps one terminal cell.
// These are coarse feel knobs; they only affect pointer speed, not correctness.
#define MOUSE_BIOS_SENS 4
#define MOUSE_UEFI_SENS 4

// Move the pointer by a number of cells, clamped to the visible terminal.
static void mouse_move_cells(long dx, long dy) {
    long x = (long)mouse_x + dx;
    long y = (long)mouse_y + dy;

    if (x < 0) x = 0;
    if (y < 0) y = 0;

    if (terms_i != 0 && terms[0] != NULL) {
        if (terms[0]->cols != 0 && x > (long)terms[0]->cols - 1)
            x = (long)terms[0]->cols - 1;
        if (terms[0]->rows != 0 && y > (long)terms[0]->rows - 1)
            y = (long)terms[0]->rows - 1;
    }

    mouse_x = (size_t)x;
    mouse_y = (size_t)y;
}

// Fold a raw axis delta into its accumulator and return whole cells to step.
static long mouse_accum_axis(long *accum, long raw, long sens) {
    if (sens < 1) sens = 1;
    *accum += raw;
    long cells = *accum / sens;
    *accum -= cells * sens;
    return cells;
}

static void mouse_center(void) {
    if (terms_i != 0 && terms[0] != NULL) {
        mouse_x = terms[0]->cols / 2;
        mouse_y = terms[0]->rows / 2;
    }
}

#if defined (BIOS)

extern volatile uint8_t mouse_pkt_buf[64];
extern volatile uint8_t mouse_pkt_head;
extern volatile uint8_t mouse_pkt_tail;
extern void int_74_isr(void);

// PS/2 packet length: 3 normally, 4 once the wheel (IntelliMouse) is enabled.
static int mouse_pkt_len = 3;
static uint32_t mouse_old_vec = 0;

// --- 8042 controller helpers (used only during init, with IRQs masked) ---

static bool ps2_wait_write(void) {
    for (int i = 0; i < 100000; i++)
        if (!(inb(0x64) & 0x02))
            return true;
    return false;
}

static bool ps2_wait_read(void) {
    for (int i = 0; i < 100000; i++)
        if (inb(0x64) & 0x01)
            return true;
    return false;
}

static int ps2_read(void) {
    if (!ps2_wait_read())
        return -1;
    return inb(0x60);
}

static bool ps2_cmd(uint8_t cmd) {
    if (!ps2_wait_write())
        return false;
    outb(0x64, cmd);
    return true;
}

// Send a byte to the auxiliary device and expect its 0xFA acknowledgement.
static bool ps2_aux_write(uint8_t val) {
    if (!ps2_wait_write())
        return false;
    outb(0x64, 0xd4);
    if (!ps2_wait_write())
        return false;
    outb(0x60, val);
    return ps2_read() == 0xfa;
}

static bool ps2_set_rate(uint8_t rate) {
    return ps2_aux_write(0xf3) && ps2_aux_write(rate);
}

// The classic knock that switches a wheel mouse into 4-byte report mode.
static bool ps2_enable_wheel(void) {
    if (!ps2_set_rate(200)) return false;
    if (!ps2_set_rate(100)) return false;
    if (!ps2_set_rate(80))  return false;
    if (!ps2_aux_write(0xf2)) return false; // get device ID
    int id = ps2_read();
    return id == 3;
}

static bool mouse_bios_init(void) {
    // Enable the auxiliary device port.
    ps2_cmd(0xa8);

    // Read the controller command byte, enable the IRQ12 line and the aux
    // clock, then write it back.
    if (!ps2_cmd(0x20))
        return false;
    int cb = ps2_read();
    if (cb < 0)
        return false;
    cb |= 0x02;   // enable aux (IRQ12) interrupt
    cb &= ~0x20;  // clear "aux clock disabled"
    if (!ps2_cmd(0x60))
        return false;
    if (!ps2_wait_write())
        return false;
    outb(0x60, (uint8_t)cb);

    if (!ps2_aux_write(0xf6))   // restore default parameters
        return false;

    mouse_pkt_len = ps2_enable_wheel() ? 4 : 3;

    // A modest report rate keeps the ring buffer comfortable between polls.
    ps2_set_rate(60);

    if (!ps2_aux_write(0xf4))   // enable data reporting (start streaming)
        return false;

    return true;
}

int mouse_bios_poll(void) {
    if (!mouse_available)
        return 0;

    bool left_edge = false, right_edge = false;
    long scroll = 0;
    bool moved = false;

    static uint8_t pkt[4];
    static int pkt_idx = 0;

    while ((uint8_t)(mouse_pkt_head - mouse_pkt_tail) != 0) {
        uint8_t b = mouse_pkt_buf[mouse_pkt_tail & 0x3f];
        mouse_pkt_tail++;

        // Bit 3 of the first byte is always set; use it to resync if we ever
        // start reading mid-packet (e.g. a stale byte left in the 8042).
        if (pkt_idx == 0 && !(b & 0x08))
            continue;

        pkt[pkt_idx++] = b;
        if (pkt_idx < mouse_pkt_len)
            continue;
        pkt_idx = 0;

        uint8_t flags = pkt[0];

        int dx = pkt[1];
        int dy = pkt[2];
        if (flags & 0x10) dx -= 256;   // X sign bit
        if (flags & 0x20) dy -= 256;   // Y sign bit
        if (flags & 0xc0) { dx = 0; dy = 0; }  // X/Y overflow -> drop motion

        bool left  = flags & 0x01;
        bool right = flags & 0x02;
        if (left  && !prev_left)  left_edge  = true;
        if (right && !prev_right) right_edge = true;
        prev_left  = left;
        prev_right = right;

        if (mouse_pkt_len == 4)
            scroll += (int8_t)pkt[3];

        // PS/2 reports +Y as upward; the screen's Y grows downward.
        long cx = mouse_accum_axis(&accum_x,  dx, MOUSE_BIOS_SENS);
        long cy = mouse_accum_axis(&accum_y, -dy, MOUSE_BIOS_SENS);
        if (cx != 0 || cy != 0) {
            mouse_move_cells(cx, cy);
            moved = true;
        }
    }

    if (left_edge)  return GETCHAR_MOUSE_LEFT;
    if (right_edge) return GETCHAR_MOUSE_RIGHT;
    if (scroll < 0) return GETCHAR_CURSOR_UP;    // wheel away from user
    if (scroll > 0) return GETCHAR_CURSOR_DOWN;  // wheel toward user
    if (moved)      return GETCHAR_MOUSE_MOTION;
    return 0;
}

#endif

#if defined (UEFI)

static long lmax(long a, long b) { return a > b ? a : b; }

static EFI_SIMPLE_POINTER_PROTOCOL *spp = NULL;

static bool mouse_uefi_init(void) {
    EFI_GUID spp_guid = EFI_SIMPLE_POINTER_PROTOCOL_GUID;
    EFI_STATUS status = gBS->LocateProtocol(&spp_guid, NULL, (void **)&spp);
    if (status != EFI_SUCCESS || spp == NULL) {
        spp = NULL;
        return false;
    }
    spp->Reset(spp, false);
    return true;
}

EFI_EVENT mouse_uefi_wait_event(void) {
    return spp != NULL ? spp->WaitForInput : NULL;
}

int mouse_uefi_read(void) {
    if (spp == NULL)
        return 0;

    EFI_SIMPLE_POINTER_STATE state;
    if (spp->GetState(spp, &state) != EFI_SUCCESS)
        return 0;

    bool left  = state.LeftButton;
    bool right = state.RightButton;
    bool left_edge  = left  && !prev_left;
    bool right_edge = right && !prev_right;
    prev_left  = left;
    prev_right = right;

    // Resolution is reported in counts/mm; aim for roughly two cells per mm so
    // the pointer crosses the screen with a comfortable hand movement.
    long sensx = MOUSE_UEFI_SENS, sensy = MOUSE_UEFI_SENS;
    if (spp->Mode != NULL) {
        if (spp->Mode->ResolutionX > 0) sensx = lmax(1, (long)spp->Mode->ResolutionX / 2);
        if (spp->Mode->ResolutionY > 0) sensy = lmax(1, (long)spp->Mode->ResolutionY / 2);
    }

    bool moved = false;
    long cx = mouse_accum_axis(&accum_x, state.RelativeMovementX, sensx);
    long cy = mouse_accum_axis(&accum_y, state.RelativeMovementY, sensy);
    if (cx != 0 || cy != 0) {
        mouse_move_cells(cx, cy);
        moved = true;
    }

    if (left_edge)  return GETCHAR_MOUSE_LEFT;
    if (right_edge) return GETCHAR_MOUSE_RIGHT;
    if (state.RelativeMovementZ < 0) return GETCHAR_CURSOR_UP;
    if (state.RelativeMovementZ > 0) return GETCHAR_CURSOR_DOWN;
    if (moved)      return GETCHAR_MOUSE_MOTION;
    return 0;
}

#endif

void mouse_init(void) {
    // Idempotent, and re-armable: _menu rewinds globals (clearing
    // mouse_available) when it is re-entered after a failed boot, so this will
    // probe again and bring the pointer back up.
    if (mouse_available)
        return;

    // A pointer is only meaningful on a real screen, not a serial console.
    if (serial)
        return;

#if defined (BIOS)
    if (!mouse_bios_init())
        return;

    // Take over the real-mode IRQ12 vector. This is harmless while we run in
    // protected mode (interrupts are masked there); it only fires during the
    // real-mode keyboard-polling window.
    mouse_old_vec = *(volatile uint32_t *)(uintptr_t)(0x74 * 4);
    *(volatile uint32_t *)(uintptr_t)(0x74 * 4) = (uint32_t)(uintptr_t)&int_74_isr;

    pic_set_mask(2, false);    // cascade line for the slave PIC
    pic_set_mask(12, false);   // mouse

    mouse_available = true;
#elif defined (UEFI)
    if (!mouse_uefi_init())
        return;
    mouse_available = true;
#endif

    mouse_center();
}

void mouse_deinit(void) {
    if (!mouse_available)
        return;

#if defined (BIOS)
    pic_set_mask(12, true);
    ps2_aux_write(0xf5);   // disable data reporting
    *(volatile uint32_t *)(uintptr_t)(0x74 * 4) = mouse_old_vec;
#endif

    mouse_available = false;
}
