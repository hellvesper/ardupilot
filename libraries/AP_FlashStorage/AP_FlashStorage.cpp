/*
   Please contribute your ideas! See https://ardupilot.org/dev for details

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_FlashStorage/AP_FlashStorage.h>
#include <AP_Math/AP_Math.h>
#include <AP_InternalError/AP_InternalError.h>
#include <stdio.h>

#define FLASHSTORAGE_DEBUG 0

#if FLASHSTORAGE_DEBUG
#define debug(fmt, args...)  do { printf(fmt, ##args); } while(0)
#else
#define debug(fmt, args...)  do { } while(0)
#endif

// constructor.
AP_FlashStorage::AP_FlashStorage(uint8_t *_mem_buffer,
                                 uint32_t _flash_sector_size,
                                 FlashWrite _flash_write,
                                 FlashRead _flash_read,
                                 FlashErase _flash_erase,
                                 FlashEraseOK _flash_erase_ok) :
    mem_buffer(_mem_buffer),
    flash_sector_size(_flash_sector_size),
    flash_write(_flash_write),
    flash_read(_flash_read),
    flash_erase(_flash_erase),
    flash_erase_ok(_flash_erase_ok) {}

// initialise storage
bool AP_FlashStorage::init(void)
{
    debug("%s:%d start storage init \n", __PRETTY_FUNCTION__, __LINE__);
    

    // start with empty memory buffer
    memset(mem_buffer, 0, storage_size);

    // find state of sectors
    struct sector_header header[2];

    // read headers and possibly initialise if bad signature
    debug("%s:%d read headers and possibly initialise if bad signature \n", __PRETTY_FUNCTION__, __LINE__);
    for (uint8_t i=0; i<2; i++) {
        if (!flash_read(i, 0, (uint8_t *)&header[i], sizeof(header[i]))) {
            debug("%s:%d Err, can't read header \n", __PRETTY_FUNCTION__, __LINE__);
            return false;
        }
        bool bad_header = !header[i].signature_ok();
        debug("%s:%d bad_header: %s\n", __PRETTY_FUNCTION__, __LINE__, bad_header ? "true" : "false");
        enum SectorState state = header[i].get_state();
        if (state != SECTOR_STATE_AVAILABLE &&
            state != SECTOR_STATE_IN_USE &&
            state != SECTOR_STATE_FULL) {
            bad_header = true;
            debug("%s:%d bad_header is True \n", __PRETTY_FUNCTION__, __LINE__);
        }

        // initialise if bad header
        debug("%s:%d initialise if bad header \n", __PRETTY_FUNCTION__, __LINE__);
        if (bad_header) {
            debug("%s:%d bad_header=true, erase_all \n", __PRETTY_FUNCTION__, __LINE__);
            return erase_all();
        }
    }

    // work out the first sector to read from using sector states
    debug("%s:%d work out the first sector to read from using sector states \n", __PRETTY_FUNCTION__, __LINE__);
    enum SectorState states[2] {header[0].get_state(), header[1].get_state()};
    uint8_t first_sector;

    if (states[0] == states[1]) {
        if (states[0] != SECTOR_STATE_AVAILABLE) {
            return erase_all();
        }
        first_sector = 0;
    } else if (states[0] == SECTOR_STATE_FULL) {
        first_sector = 0;
    } else if (states[1] == SECTOR_STATE_FULL) {
        first_sector = 1;
    } else if (states[0] == SECTOR_STATE_IN_USE) {
        first_sector = 0;
    } else if (states[1] == SECTOR_STATE_IN_USE) {
        first_sector = 1;
    } else {
        // doesn't matter which is first
        first_sector = 0;
    }

    // load data from any current sectors
    debug("%s:%d load data from any current sectors \n", __PRETTY_FUNCTION__, __LINE__);
    for (uint8_t i=0; i<2; i++) {
        uint8_t sector = (first_sector + i) & 1;
        if (states[sector] == SECTOR_STATE_IN_USE ||
            states[sector] == SECTOR_STATE_FULL) {
            if (!load_sector(sector)) {
                return erase_all();
            }
        }
    }

    // clear any write error
    debug("%s:%d clear any write error \n", __PRETTY_FUNCTION__, __LINE__);
    write_error = false;
    reserved_space = 0;
    
    // if the first sector is full then write out all data so we can erase it
    debug("%s:%d if the first sector is full then write out all data so we can erase it \n", __PRETTY_FUNCTION__, __LINE__);
    if (states[first_sector] == SECTOR_STATE_FULL) {
        current_sector = first_sector ^ 1;
        if (!write_all()) {
            return erase_all();
        }
    }

    // erase any sectors marked full
    debug("%s:%d erase any sectors marked full \n", __PRETTY_FUNCTION__, __LINE__);
    for (uint8_t i=0; i<2; i++) {
        if (states[i] == SECTOR_STATE_FULL) {
            if (!erase_sector(i, true)) {
                return false;
            }
        }
    }

    reserved_space = 0;
    
    // ready to use
    debug("%s:%d ready to use \n", __PRETTY_FUNCTION__, __LINE__);
    return true;
}



// switch full sector - should only be called when safe to have CPU
// offline for considerable periods as an erase will be needed
bool AP_FlashStorage::switch_full_sector(void)
{
    debug("running switch_full_sector()\n");

    if (in_switch_full_sector) {
        INTERNAL_ERROR(AP_InternalError::error_t::switch_full_sector_recursion);
        return false;
    }
    in_switch_full_sector = true;
    bool ret = protected_switch_full_sector();
    in_switch_full_sector = false;
    return ret;
}

// protected_switch_full_sector is protected by switch_full_sector to
// avoid an infinite recursion problem; switch_full_sector calls
// write() which can call switch_full_sector.  This has been seen in
// practice, and while it might be caused by corruption... corruption
// happens.
bool AP_FlashStorage::protected_switch_full_sector(void)
{
    // clear any write error
    write_error = false;
    reserved_space = 0;
    
    if (!write_all()) {
        return false;
    }

    if (!erase_sector(current_sector ^ 1, true)) {
        return false;
    }

    return switch_sectors();
}

// write some data to virtual EEPROM
bool AP_FlashStorage::write(uint16_t offset, uint16_t length)
{
    if (write_error) {
        return false;
    }
    //debug("write at %u for %u write_offset=%u\n", offset, length, write_offset);
    
    while (length > 0) {
        uint8_t n = max_write;
#if AP_FLASHSTORAGE_TYPE != AP_FLASHSTORAGE_TYPE_H7 && AP_FLASHSTORAGE_TYPE != AP_FLASHSTORAGE_TYPE_G4
        if (length < n) {
            n = length;
        }
#endif

        const uint32_t space_available = flash_sector_size - write_offset;
        const uint32_t space_required = sizeof(struct block_header) + max_write + reserved_space;
        if (space_available < space_required) {
            if (!switch_sectors()) {
                if (!flash_erase_ok()) {
                    return false;
                }
                if (!switch_full_sector()) {
                    return false;                    
                }
            }
        }

        struct PACKED {
            struct block_header header;
            uint8_t data[max_write];
        } blk;

        blk.header.state = BLOCK_STATE_WRITING;
        blk.header.block_num = offset / block_size;
        blk.header.num_blocks_minus_one = ((n + (block_size - 1)) / block_size)-1;

        uint16_t block_ofs = blk.header.block_num*block_size;
        uint16_t block_nbytes = (blk.header.num_blocks_minus_one+1)*block_size;

        memcpy(blk.data, &mem_buffer[block_ofs], block_nbytes);

#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F4
        if (!flash_write(current_sector, write_offset, (uint8_t*)&blk.header, sizeof(blk.header))) {
            return false;
        }
        if (!flash_write(current_sector, write_offset+sizeof(blk.header), blk.data, block_nbytes)) {
            return false;
        }
        blk.header.state = BLOCK_STATE_VALID;
        if (!flash_write(current_sector, write_offset, (uint8_t*)&blk.header, sizeof(blk.header))) {
            return false;
        }
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F1
        blk.header.state = BLOCK_STATE_VALID;
        if (!flash_write(current_sector, write_offset, (uint8_t*)&blk, sizeof(blk.header) + block_nbytes)) {
            return false;
        }
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7 || AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
        blk.header.state = BLOCK_STATE_VALID;
        if (!flash_write(current_sector, write_offset, (uint8_t*)&blk, sizeof(blk.header) + max_write)) {
            return false;
        }
#endif

        write_offset += sizeof(blk.header) + block_nbytes;

        uint8_t n2 = block_nbytes - (offset % block_size);
        //debug("write_block at %u for %u n2=%u\n", block_ofs, block_nbytes, n2);
        if (n2 > length) {
            break;
        }
        offset += n2;
        length -= n2;
    }

    // debug("write_offset %u\n", write_offset);

    // handle wrap to next sector
    // write data
    // write header word
    return true;
}

/*
  load all data from a flash sector into mem_buffer
 */
bool AP_FlashStorage::load_sector(uint8_t sector)
{
    uint32_t ofs = sizeof(sector_header);
    while (ofs < flash_sector_size - sizeof(struct block_header)) {
        struct block_header header;
        if (!flash_read(sector, ofs, (uint8_t *)&header, sizeof(header))) {
            return false;
        }
        enum BlockState state = (enum BlockState)header.state;

        switch (state) {
        case BLOCK_STATE_AVAILABLE:
            // we've reached the end
            write_offset = ofs;
            return true;

        case BLOCK_STATE_WRITING: {
            /*
              we were interrupted while writing a block. We can't
              re-use the data in this block as it may have some bits
              that are not set to 1, so by flash rules can't be set to
              an arbitrary value. So we skip over this block, leaving
              a gap. The gap size is limited to (7+1)*8=64 bytes. That
              gap won't be recovered until we next do an erase of this
              sector
             */
            uint16_t block_nbytes = (header.num_blocks_minus_one+1)*block_size;
            ofs += block_nbytes + sizeof(header);
            break;
        }
            
        case BLOCK_STATE_VALID: {
            uint16_t block_nbytes = (header.num_blocks_minus_one+1)*block_size;
            uint16_t block_ofs = header.block_num*block_size;
            if (block_ofs + block_nbytes > storage_size) {
                // the data is invalid (out of range)
                return false;
            }
            if (!flash_read(sector, ofs+sizeof(header), &mem_buffer[block_ofs], block_nbytes)) {
                return false;
            }
            //debug("read at %u for %u\n", block_ofs, block_nbytes);
            ofs += block_nbytes + sizeof(header);
            break;
        }
        default:
            // invalid state
            return false;
        }
#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7
        // offsets must be advanced to a multiple of 32 on H7
        ofs = (ofs + 31U) & ~31U;
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
        // offsets must be advanced to a multiple of 8 on G4
        ofs = (ofs + 7U) & ~7U;
#endif
    }
    write_offset = ofs;
    return true;
}

/*
  erase one sector
 */
bool AP_FlashStorage::erase_sector(uint8_t sector, bool mark_available)
{
    debug("%s -> %s:%d erase one sector: %u mark: %s  \n", __FILE__, __PRETTY_FUNCTION__, __LINE__, sector, mark_available ? "true" : "false");
    if (!flash_erase(sector)) {
        return false;
    }
    debug("%s -> %s:%d erase sector %u: ok\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, sector);
    if (!mark_available) {
        return true;
    }
    struct sector_header header;
    header.set_state(SECTOR_STATE_AVAILABLE);
    debug("%s -> %s:%d writing sector %u: ok\n", __FILE__, __PRETTY_FUNCTION__, __LINE__, sector);
    return flash_write(sector, 0, (const uint8_t *)&header, sizeof(header));
}

/*
  erase both sectors
 */
bool AP_FlashStorage::erase_all(void)
{
    write_error = false;

    current_sector = 0;
    write_offset = sizeof(struct sector_header);
    debug("%s -> %s:%d erase sector 0 \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
    if (!erase_sector(0, current_sector!=0)) {
        return false;
    }
    debug("%s -> %s:%d erase sector 1 \n", __FILE__, __PRETTY_FUNCTION__, __LINE__);
    if (!erase_sector(1, current_sector!=1)) {
        return false;
    }
    
    // mark current sector as in-use
    debug("%s:%d mark current sector as in-use \n", __PRETTY_FUNCTION__, __LINE__);
    debug("%s:%d init header \n", __PRETTY_FUNCTION__, __LINE__);
    struct sector_header header;
    debug("%s:%d set state \n", __PRETTY_FUNCTION__, __LINE__);
    header.set_state(SECTOR_STATE_IN_USE);
    debug("%s:%d flash write \n", __PRETTY_FUNCTION__, __LINE__);
    return flash_write(current_sector, 0, (const uint8_t *)&header, sizeof(header));    
}

/*
  write all of mem_buffer to current sector
 */
bool AP_FlashStorage::write_all()
{
    debug("write_all to sector %u at %u with reserved_space=%u\n",
           current_sector, write_offset, reserved_space);
    for (uint16_t ofs=0; ofs<storage_size; ofs += max_write) {
        // local variable needed to overcome problem with MIN() macro and -O0
        const uint8_t max_write_local = max_write;
        uint8_t n = MIN(max_write_local, storage_size-ofs);
        if (!all_zero(ofs, n)) {
            if (!write(ofs, n)) {
                return false;
            }
        }
    }
    return true;
}

// return true if all bytes are zero
bool AP_FlashStorage::all_zero(uint16_t ofs, uint16_t size)
{
    while (size--) {
        if (mem_buffer[ofs++] != 0) {
            return false;
        }
    }
    return true;
}

// switch to next sector for writing
bool AP_FlashStorage::switch_sectors(void)
{
    if (reserved_space != 0) {
        // other sector is already full
        debug("both sectors are full\n");
        return false;
    }

    struct sector_header header;

    uint8_t new_sector = current_sector ^ 1;
    debug("switching to sector %u\n", new_sector);
    
    // check sector is available
    if (!flash_read(new_sector, 0, (uint8_t *)&header, sizeof(header))) {
        return false;
    }
    if (!header.signature_ok()) {
        write_error = true;
        return false;
    }
    if (SECTOR_STATE_AVAILABLE != header.get_state()) {
        write_error = true;
        debug("new sector unavailable; state=0x%02x\n", (unsigned)header.get_state());
        return false;
    }

    // mark current sector as full. This needs to be done before we
    // mark the new sector as in-use so that a power failure between
    // the two steps doesn't leave us with an erase on the
    // reboot. Thanks to night-ghost for spotting this.
    header.set_state(SECTOR_STATE_FULL);
    if (!flash_write(current_sector, 0, (const uint8_t *)&header, sizeof(header))) {
        return false;
    }

    // mark new sector as in-use
    header.set_state(SECTOR_STATE_IN_USE);
    if (!flash_write(new_sector, 0, (const uint8_t *)&header, sizeof(header))) {
        return false;
    }

    // switch sectors
    current_sector = new_sector;
        
    // we need to reserve some space in next sector to ensure we can successfully do a
    // full write out on init()
    reserved_space = reserve_size;
    
    write_offset = sizeof(header);
    return true;    
}

/*
  re-initialise, using current mem_buffer
 */
bool AP_FlashStorage::re_initialise(void)
{
    if (!flash_erase_ok()) {
        return false;
    }
    if (!erase_all()) {
        return false;        
    }
    return write_all();
}

#if AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_H7
/*
  H7 specific sector header functions
 */
bool AP_FlashStorage::sector_header::signature_ok(void) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(pad1); i++) {
        if (pad1[i] != 0xFFFFFFFFU || pad2[i] != 0xFFFFFFFFU || pad3[i] != 0xFFFFFFFFU) {
            return false;
        }
    }
    return signature1 == signature;
}

AP_FlashStorage::SectorState AP_FlashStorage::sector_header::get_state(void) const
{
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFFF &&
        state3 == 0xFFFFFFFF &&
        signature1 == signature &&
        signature2 == 0xFFFFFFFF &&
        signature3 == 0xFFFFFFFF) {
        return SECTOR_STATE_AVAILABLE;
    }
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFF2 &&
        state3 == 0xFFFFFFFF &&
        signature1 == signature &&
        signature2 == signature &&
        signature3 == 0xFFFFFFFF) {
        return SECTOR_STATE_IN_USE;
    }
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFF2 &&
        state3 == 0xFFFFFFF3 &&
        signature1 == signature &&
        signature2 == signature &&
        signature3 == signature) {
        return SECTOR_STATE_FULL;
    }
    return SECTOR_STATE_INVALID;
}

void AP_FlashStorage::sector_header::set_state(SectorState state)
{
    memset(pad1, 0xff, sizeof(pad1));
    memset(pad2, 0xff, sizeof(pad2));
    memset(pad3, 0xff, sizeof(pad3));
    switch (state) {
    case SECTOR_STATE_AVAILABLE:
        signature1 = signature;
        signature2 = 0xFFFFFFFF;
        signature3 = 0xFFFFFFFF;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFFF;
        state3 = 0xFFFFFFFF;
        break;
    case SECTOR_STATE_IN_USE:
        signature1 = signature;
        signature2 = signature;
        signature3 = 0xFFFFFFFF;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFF2;
        state3 = 0xFFFFFFFF;
        break;
    case SECTOR_STATE_FULL:
        signature1 = signature;
        signature2 = signature;
        signature3 = signature;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFF2;
        state3 = 0xFFFFFFF3;
        break;
    default:
        break;
    }
}

#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_G4
/*
  G4 specific sector header functions
 */
bool AP_FlashStorage::sector_header::signature_ok(void) const
{
    return signature1 == signature;
}

AP_FlashStorage::SectorState AP_FlashStorage::sector_header::get_state(void) const
{
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFFF &&
        state3 == 0xFFFFFFFF &&
        signature1 == signature &&
        signature2 == 0xFFFFFFFF &&
        signature3 == 0xFFFFFFFF) {
        return SECTOR_STATE_AVAILABLE;
    }
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFF2 &&
        state3 == 0xFFFFFFFF &&
        signature1 == signature &&
        signature2 == signature &&
        signature3 == 0xFFFFFFFF) {
        return SECTOR_STATE_IN_USE;
    }
    if (state1 == 0xFFFFFFF1 &&
        state2 == 0xFFFFFFF2 &&
        state3 == 0xFFFFFFF3 &&
        signature1 == signature &&
        signature2 == signature &&
        signature3 == signature) {
        return SECTOR_STATE_FULL;
    }
    return SECTOR_STATE_INVALID;
}

void AP_FlashStorage::sector_header::set_state(SectorState state)
{
    switch (state) {
    case SECTOR_STATE_AVAILABLE:
        signature1 = signature;
        signature2 = 0xFFFFFFFF;
        signature3 = 0xFFFFFFFF;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFFF;
        state3 = 0xFFFFFFFF;
        break;
    case SECTOR_STATE_IN_USE:
        signature1 = signature;
        signature2 = signature;
        signature3 = 0xFFFFFFFF;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFF2;
        state3 = 0xFFFFFFFF;
        break;
    case SECTOR_STATE_FULL:
        signature1 = signature;
        signature2 = signature;
        signature3 = signature;
        state1 = 0xFFFFFFF1;
        state2 = 0xFFFFFFF2;
        state3 = 0xFFFFFFF3;
        break;
    default:
        break;
    }
}

#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F1
/*
  F1/F3 specific sector header functions
 */
bool AP_FlashStorage::sector_header::signature_ok(void) const
{
    return signature1 == signature;
}

AP_FlashStorage::SectorState AP_FlashStorage::sector_header::get_state(void) const
{
    if (state1 == 0xFFFFFFFF) {
        return SECTOR_STATE_AVAILABLE;
    }
    if (state1 == 0xFFFFFFF1) {
        return SECTOR_STATE_IN_USE;
    }
    if (state1 == 0xFFF2FFF1) {
        return SECTOR_STATE_FULL;
    }
    return SECTOR_STATE_INVALID;
}

void AP_FlashStorage::sector_header::set_state(SectorState state)
{
    signature1 = signature;
    switch (state) {
    case SECTOR_STATE_AVAILABLE:
        state1 = 0xFFFFFFFF;
        break;
    case SECTOR_STATE_IN_USE:
        state1 = 0xFFFFFFF1;
        break;
    case SECTOR_STATE_FULL:
        state1 = 0xFFF2FFF1;
        break;
    default:
        break;
    }
}
#elif AP_FLASHSTORAGE_TYPE == AP_FLASHSTORAGE_TYPE_F4
/*
  F4 specific sector header functions
 */
bool AP_FlashStorage::sector_header::signature_ok(void) const
{
    debug("%s:%d signature1 0x%06X\n",  __PRETTY_FUNCTION__, __LINE__, signature1);
    debug("%s:%d signature1 %u\n",      __PRETTY_FUNCTION__, __LINE__, signature1);
    debug("%s:%d signature: 0x%06X\n",  __PRETTY_FUNCTION__, __LINE__, signature);
    debug("%s:%d signature: %u\n",      __PRETTY_FUNCTION__, __LINE__, signature);
    return signature1 == signature;
}

AP_FlashStorage::SectorState AP_FlashStorage::sector_header::get_state(void) const
{
    if (state1 == 0xFF) {
        return SECTOR_STATE_AVAILABLE;
    }
    if (state1 == 0xFE) {
        return SECTOR_STATE_IN_USE;
    }
    if (state1 == 0xFC) {
        return SECTOR_STATE_FULL;
    }
    return SECTOR_STATE_INVALID;
}

void AP_FlashStorage::sector_header::set_state(SectorState state)
{
    signature1 = signature;
    debug("%s:%d signature1 0x%06X\n",  __PRETTY_FUNCTION__, __LINE__, signature1);
    debug("%s:%d signature1 %u\n",      __PRETTY_FUNCTION__, __LINE__, signature1);
    debug("%s:%d signature: 0x%06X\n",  __PRETTY_FUNCTION__, __LINE__, signature);
    debug("%s:%d signature: %u\n",      __PRETTY_FUNCTION__, __LINE__, signature);
    switch (state) {
    case SECTOR_STATE_AVAILABLE:
        state1 = 0xFF;
        break;
    case SECTOR_STATE_IN_USE:
        state1 = 0xFE;
        break;
    case SECTOR_STATE_FULL:
        state1 = 0xFC;
        break;
    default:
        break;
    }
}
#endif
