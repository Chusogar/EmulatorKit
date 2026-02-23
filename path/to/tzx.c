// Content from commit 78202e049cc973f148ecb996c927c231d93cbf7d with tzx.c updates

#include <stdint.h>

void gen_read_pilot_entry(...) {
    // Change output type
    uint32_t rep = ...;  // Logic to determine rep
    if (rep == 0) rep = 65536;
    // Continued logic...
}

void gen_advance_symbol(...) {
    // Schedule next_edge_at based on the previous next_edge_at
    next_edge_at = edge_time;  // Avoid drift by using edge_time instead of t_now
}

// Additional code with changed gen_pilot_rep_left to uint32_t and adjusted pilot repetition decrement logic

