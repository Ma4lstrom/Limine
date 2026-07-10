#ifndef LIB__MOUSE_H__
#define LIB__MOUSE_H__

#include <stddef.h>
#include <stdbool.h>

// Whether a pointer device was detected and is currently active.
extern bool mouse_available;

// Virtual pointer position, expressed in terminal cells (terms[0]).
extern size_t mouse_x, mouse_y;

// Probe for a pointer device and start it. Safe to call multiple times; only
// the first call does anything. No-op on serial consoles.
void mouse_init(void);

// Return the hardware to the state it was in before mouse_init(). Called right
// before handing control to a kernel so we don't leave IRQs/devices armed.
void mouse_deinit(void);

#if defined (BIOS)
// Drain the PS/2 packet ring buffer filled by the IRQ12 handler and translate
// it into a single GETCHAR_* event (or 0 if nothing actionable happened).
int mouse_bios_poll(void);
#endif

#if defined (UEFI)
#include <efi.h>
// The Simple Pointer's WaitForInput event, so the input loop can block on it
// alongside the keyboard. NULL if no pointer is available.
EFI_EVENT mouse_uefi_wait_event(void);
// Read the current pointer state and translate it into a GETCHAR_* event
// (or 0 if nothing actionable happened).
int mouse_uefi_read(void);
#endif

#endif
