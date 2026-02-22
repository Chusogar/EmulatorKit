// Add printf logging for each TZX block

// Helper function to get the block name
const char* tzx_block_name(uint8_t block_id) {
    switch (block_id) {
        case 0x10: return "Block Type 0x10";
        case 0x11: return "Block Type 0x11";
        case 0x12: return "Block Type 0x12";
        case 0x13: return "Block Type 0x13";
        case 0x14: return "Block Type 0x14";
        case 0x15: return "Block Type 0x15";
        case 0x18: return "Block Type 0x18";
        case 0x19: return "Block Type 0x19";
        case 0x20: return "Control Block 0x20";
        case 0x23: return "Control Block 0x23";
        case 0x2B: return "Control Block 0x2B";
        default: return "Unknown Block";
    }
}

// Helper function to dump parameters from buffer
void tzx_dump_block_params_from_buf(uint8_t block_id, const uint8_t* buffer, size_t length) {
    printf("Loading Block: %s\n", tzx_block_name(block_id));
    // Add logic to print specific parameters based on block_id and manage parameters
}

// Helper function to dump current parameters for the block
void tzx_dump_block_params_current(uint8_t block_id) {
    printf("Playing Block: %s\n", tzx_block_name(block_id));
    // Add logic to print specific parameters based on block_id
}

void tzx_build_block_index(...) {
    // Existing code
    // Add printf logging
    printf("Block Indexing: Total Blocks = %d\n", total_blocks);
    tzx_dump_block_params_current(block_id);
    // Rest of the method
}

void tzx_advance_to(...) {
    // Existing code
    printf("Playing Block: %s\n", tzx_block_name(block_id));
    // Rest of the method
}
