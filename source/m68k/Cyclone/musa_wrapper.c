// Attempt to expose the functionality of Cyclone 68000 with the API of Musashi
// Copyright (c) 2012 Darren "lifning" Alton (definelightning (at) gmail.com)

#include <setjmp.h>
#include <string.h>
#include <stdio.h>

#include "Cyclone.h"
#include "m68kconf.h"
#include "m68k.h"

#define MUSASHI_CYCLES_UPDATE 0

extern int vdp_68k_irq_ack(int int_level); // "vdp_ctrl.h"
extern int scd_68k_irq_ack(int int_level); // "cd_hw/scd.h"

static int m_irq_delay_lock = 0, s_irq_delay_lock = 0;

m68ki_cpu_core m68k, s68k;
struct Cyclone m_cyclone, s_cyclone;

#define MM(a) ((a>>16) & 0xFF)

#define RWDEFS(core, bits) \
unsigned int cyclone_##core##68k_read##bits(unsigned int a) \
{ \
  /*fprintf(stderr, "cyclone_" #core "68k_read" #bits "(...)\n");*/ \
  if ((core##68k).memory_map[MM(a)].read##bits) \
    return (core##68k).memory_map[MM(a)].read##bits(a); \
  return *(uint##bits *)((core##68k).memory_map[MM(a)].base + (a & 0xFFFF)); \
} \
void cyclone_##core##68k_write##bits(unsigned int a, uint##bits d) \
{ \
  /*fprintf(stderr, "cyclone_" #core "68k_write" #bits "(...)\n");*/ \
  if ((core##68k).memory_map[MM(a)].write##bits) \
    (core##68k).memory_map[MM(a)].write##bits(a, d); \
  *(uint##bits *)((core##68k).memory_map[MM(a)].base + (a & 0xFFFF)) = d; \
}

RWDEFS(m, 8)
RWDEFS(s, 8)
RWDEFS(m, 16)
RWDEFS(s, 16)

#define RWDEFS32PC(core) \
unsigned int cyclone_##core##68k_read32(unsigned int a) \
{ \
  /*fprintf(stderr, "cyclone_" #core "68k_read32(...)\n");*/ \
  return (cyclone_##core##68k_read16(a) << 16) | cyclone_##core##68k_read16(a+2); \
} \
void cyclone_##core##68k_write32(unsigned int a, uint32 d) \
{ \
  /*fprintf(stderr, "cyclone_" #core "68k_write32(...)\n");*/ \
  cyclone_##core##68k_write16(a, d >> 16); \
  cyclone_##core##68k_write16(a+2, d & 0xFFFF); \
} \
unsigned int cyclone_##core##68k_checkpc(unsigned int pc) \
{ \
  /*fprintf(stderr, "cyclone_" #core "68k_checkpc(...)\n");*/ \
  pc -= core##_cyclone.membase; \
  core##_cyclone.membase = (unsigned int)(core##68k).memory_map[MM(pc)].base; \
  return pc + core##_cyclone.membase; \
}

RWDEFS32PC(m)
RWDEFS32PC(s)

void m68k_set_int_ack_callback(int (*callback)(int int_level))
{
  m_cyclone.IrqCallback = callback;
}
void m68k_set_reset_instr_callback(void (*callback)(void))
{
  m_cyclone.ResetCallback = callback;
}

void m68k_set_tas_instr_callback(int (*callback)(void))
{
  // is this appropriate?
  CycloneSetRealTAS(!!callback);
}

void m68k_init(void)
{
  memset(&m_cyclone, 0, sizeof(m_cyclone));
  memset(&s_cyclone, 0, sizeof(s_cyclone));
  m_cyclone.checkpc = cyclone_m68k_checkpc;
  m_cyclone.read8  = m_cyclone.fetch8  = cyclone_m68k_read8;
  m_cyclone.read16 = m_cyclone.fetch16 = cyclone_m68k_read16;
  m_cyclone.read32 = m_cyclone.fetch32 = cyclone_m68k_read32;
  m_cyclone.write8  = cyclone_m68k_write8;
  m_cyclone.write16 = cyclone_m68k_write16;
  m_cyclone.write32 = cyclone_m68k_write32;
  m_cyclone.IrqCallback = vdp_68k_irq_ack;
  CycloneInit();
}

void m68k_pulse_reset(void)
{
  CycloneReset(&m_cyclone);
}

void m68k_run(unsigned int cycles)
{
#if MUSASHI_CYCLES_UPDATE
  m68k.cycles = 0;
  m68k.cycle_end = cycles;
  while (m68k.cycles < m68k.cycle_end)
  {
    m_cyclone.cycles = 0;
    CycloneRun(&m_cyclone);
    m68k.cycles -= m_cyclone.cycles;
  }
#else
  // this isn't good for timing, m68k.cycles must update during CycloneRun.
  // otherwise we'd have to have #ifdef HAVE_CYCLONE all over mem68k.c, etc.
  // ARM register r5 is cycles remaining, subtract that from m68k.cycle_end ?
  // or just run one instruction at a time (huge performance hit).
  m_cyclone.cycles = m68k.cycle_end = cycles;
  CycloneRun(&m_cyclone);
  m68k.cycles = cycles - m_cyclone.cycles;
#endif
}

void m68k_set_irq(unsigned int int_level)
{
  m_cyclone.irq = int_level;
  m68k.int_level = int_level << 8;
  // FIXME: state.c sets m68k.int_level directly, so savestates may break?
  //  is there a good way to restore it, maybe during pulse_reset?
  m68k.cycles = CycloneFlushIrq(&m_cyclone);
}

void m68k_set_irq_delay(unsigned int int_level)
{
  // prevent re-entrance
  if(!m_irq_delay_lock)
  {
    m_irq_delay_lock = 1;
    // m_cyclone.state_flags |= 1<<1; // trace
    m68k_run(0); // run one instruction
    // exception_if_trace()
    m_irq_delay_lock = 0;
  }
  m68k_set_irq(int_level);
}

void m68k_update_irq(unsigned int mask)
{
  m_cyclone.irq |= mask;
  m68k.int_level |= mask << 8;
  m68k.cycles = CycloneFlushIrq(&m_cyclone);
}

void m68k_pulse_halt(void) {  m_cyclone.state_flags |=  (1<<4);  }
void m68k_clear_halt(void) {  m_cyclone.state_flags &= ~(1<<4);  }

unsigned int m68k_get_reg(m68k_register_t reg)
{
  if (reg < 8)  return m_cyclone.d[reg];
  if (reg < 16) return m_cyclone.a[reg&7];
  switch (reg)
  {
    case M68K_REG_PC:  return m_cyclone.pc - m_cyclone.membase;
    case M68K_REG_SR:  return CycloneGetSr(&m_cyclone);
    case M68K_REG_SP:  return m_cyclone.a[7];
    // these next two might be wrong
    case M68K_REG_USP:  return (m_cyclone.srh & 0x20) ? m_cyclone.osp : m_cyclone.a[7];
    case M68K_REG_ISP:  return (m_cyclone.srh & 0x20) ? m_cyclone.a[7] : m_cyclone.osp;
#if defined(M68K_REG_PREF_ADDR) && defined(M68K_REG_PREF_DATA)
    case M68K_REG_PREF_ADDR: return m_cyclone.prev_pc - m_cyclone.membase;
    case M68K_REG_PREF_DATA: return *((uint16*)(m_cyclone.prev_pc));
#endif
    case M68K_REG_IR: return *((uint16*)(m_cyclone.pc));
    default: return 0;
  }
}

void m68k_set_reg(m68k_register_t reg, unsigned int value)
{
  if (reg < 8)  { m_cyclone.d[reg]   = value; return; }
  if (reg < 16) { m_cyclone.a[reg&7] = value; return; }
  switch (reg)
  {
    case M68K_REG_PC:  m_cyclone.pc = cyclone_m68k_checkpc(value);  break;
    case M68K_REG_SR:  CycloneSetSr(&m_cyclone, value);  break;
    case M68K_REG_SP:  m_cyclone.a[7] = value;  break;
    // these next two might be wrong
    case M68K_REG_USP:
      if (m_cyclone.srh & 0x20)
        m_cyclone.osp = value;
      else
        m_cyclone.a[7] = value;
      break;
    case M68K_REG_ISP:
      if (m_cyclone.srh & 0x20)
        m_cyclone.a[7] = value;
      else
        m_cyclone.osp = value;
      break;
#if defined(M68K_REG_PREF_ADDR) && defined(M68K_REG_PREF_DATA)
    case M68K_REG_PREF_ADDR: m_cyclone.prev_pc = value + m_cyclone.membase;
    // FIXME: should this be allowed?  can we write something more temporary?
    //case M68K_REG_PREF_DATA: *((uint16*)(m_cyclone.prev_pc)) = value;  break;
#endif
    //case M68K_REG_IR: *((uint16*)(m_cyclone.pc)) = value;  break;
    default: break;
  }
}


// --- repeat of the above, but for the s68k



void s68k_set_int_ack_callback(int (*callback)(int int_level))
{
  s_cyclone.IrqCallback = callback;
}
void s68k_set_reset_instr_callback(void (*callback)(void))
{
  s_cyclone.ResetCallback = callback;
}

void s68k_set_tas_instr_callback(int (*callback)(void))
{
  // is this appropriate?
  CycloneSetRealTAS(!!callback);
}

void s68k_init(void)
{
  memset(&s_cyclone, 0, sizeof(s_cyclone));
  memset(&s_cyclone, 0, sizeof(s_cyclone));
  s_cyclone.checkpc = cyclone_s68k_checkpc;
  s_cyclone.read8  = s_cyclone.fetch8  = cyclone_s68k_read8;
  s_cyclone.read16 = s_cyclone.fetch16 = cyclone_s68k_read16;
  s_cyclone.read32 = s_cyclone.fetch32 = cyclone_s68k_read32;
  s_cyclone.write8  = cyclone_s68k_write8;
  s_cyclone.write16 = cyclone_s68k_write16;
  s_cyclone.write32 = cyclone_s68k_write32;
  s_cyclone.IrqCallback = scd_68k_irq_ack;
  CycloneInit();
  CycloneReset(&s_cyclone);
}

void s68k_pulse_reset(void)
{
  CycloneReset(&s_cyclone);
}

void s68k_run(unsigned int cycles)
{
#if MUSASHI_CYCLES_UPDATE
  s68k.cycles = 0;
  s68k.cycle_end = cycles;
  while (s68k.cycles < s68k.cycle_end)
  {
    s_cyclone.cycles = 0;
    CycloneRun(&s_cyclone);
    s68k.cycles -= s_cyclone.cycles;
  }
#else
  // this isn't good for timing, s68k.cycles must update during CycloneRun.
  // otherwise we'd have to have #ifdef HAVE_CYCLONE all over mes68k.c, etc.
  // ARM register r5 is cycles remaining, subtract that from s68k.cycle_end ?
  // or just run one instruction at a time (huge performance hit).
  s_cyclone.cycles = s68k.cycle_end = cycles;
  CycloneRun(&s_cyclone);
  s68k.cycles = cycles - s_cyclone.cycles;
#endif
}

void s68k_set_irq(unsigned int int_level)
{
  s_cyclone.irq = int_level;
  s68k.int_level = int_level << 8;
  // FIXME: state.c sets s68k.int_level directly, so savestates may break?
  //  is there a good way to restore it, maybe during pulse_reset?
  s68k.cycles = CycloneFlushIrq(&s_cyclone);
}

void s68k_set_irq_delay(unsigned int int_level)
{
  // prevent re-entrance
  if(!s_irq_delay_lock)
  {
    s_irq_delay_lock = 1;
    // s_cyclone.state_flags |= 1<<1; // trace
    s68k_run(0); // run one instruction
    // exception_if_trace()
    s_irq_delay_lock = 0;
  }
  s68k_set_irq(int_level);
}

void s68k_update_irq(unsigned int mask)
{
  s_cyclone.irq |= mask;
  s68k.int_level |= mask << 8;
  s68k.cycles = CycloneFlushIrq(&s_cyclone);
}

void s68k_pulse_halt(void) {  s_cyclone.state_flags |=  (1<<4);  }
void s68k_clear_halt(void) {  s_cyclone.state_flags &= ~(1<<4);  }

unsigned int s68k_get_reg(m68k_register_t reg)
{
  if (reg < 8)  return s_cyclone.d[reg];
  if (reg < 16) return s_cyclone.a[reg&7];
  switch (reg)
  {
    case M68K_REG_PC:  return s_cyclone.pc - s_cyclone.membase;
    case M68K_REG_SR:  return CycloneGetSr(&s_cyclone);
    case M68K_REG_SP:  return s_cyclone.a[7];
    // these next two might be wrong
    case M68K_REG_USP:  return (s_cyclone.srh & 0x20) ? s_cyclone.osp : s_cyclone.a[7];
    case M68K_REG_ISP:  return (s_cyclone.srh & 0x20) ? s_cyclone.a[7] : s_cyclone.osp;
#ifdef M68K_REG_PREF_ADDR
    case M68K_REG_PREF_ADDR: return s_cyclone.prev_pc - s_cyclone.membase;
    case M68K_REG_PREF_DATA: return *((uint16*)(s_cyclone.prev_pc));
#endif
    case M68K_REG_IR: return *((uint16*)(s_cyclone.pc));
    default: return 0;
  }
}

void s68k_set_reg(m68k_register_t reg, unsigned int value)
{
  if (reg < 8)  { s_cyclone.d[reg]   = value; return; }
  if (reg < 16) { s_cyclone.a[reg&7] = value; return; }
  switch (reg)
  {
    case M68K_REG_PC:  s_cyclone.pc = cyclone_s68k_checkpc(value);  break;
    case M68K_REG_SR:  CycloneSetSr(&s_cyclone, value);  break;
    case M68K_REG_SP:  s_cyclone.a[7] = value;  break;
    // these next two might be wrong
    case M68K_REG_USP:
      if (s_cyclone.srh & 0x20)
        s_cyclone.osp = value;
      else
        s_cyclone.a[7] = value;
      break;
    case M68K_REG_ISP:
      if (s_cyclone.srh & 0x20)
        s_cyclone.a[7] = value;
      else
        s_cyclone.osp = value;
      break;
#ifdef M68K_REG_PREF_ADDR
    case M68K_REG_PREF_ADDR: s_cyclone.prev_pc = value + s_cyclone.membase;
    // FIXME: should this be allowed?  can we write something more temporary?
    //case M68K_REG_PREF_DATA: *((uint16*)(s_cyclone.prev_pc)) = value;  break;
#endif
    //case M68K_REG_IR: *((uint16*)(s_cyclone.pc)) = value;  break;
    default: break;
  }
}

