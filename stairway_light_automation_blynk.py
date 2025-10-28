#!/usr/bin/env python3 

# Stairway IoT — PIR + VL53L1X + digital LDR + analog SOUND (PCF8591) + WS2812 

# Day brightness 0.20, Night 0.50, Auto-off 5s 

# Blynk: V7 = Watts (instant), V6 = Wh (cumulative "power_wh") 

# Robust OFF (watchdog) + destroy() on exit 

  

import time, signal, functools, sys, collections, atexit 

print = functools.partial(print, flush=True) 

  

# ---------------- USER CONFIG ---------------- 

BLYNK_AUTH = "uCiBV9pw72HenzdaPIyXGL7E3NcAPD0S" 

  

# GPIO pins (BCM) 

PIR_PIN   = 17       # PIR DO 

LDR_PIN   = 23       # LDR DO (digital) 

SOUND_PIN = 27       # Sound DO (fallback only; analog via PCF8591 preferred) 

  

# Polarity 

PIR_ACTIVE_HIGH   = True          # most PIRs: HIGH on motion 

LDR_BRIGHT_HIGH   = False         # many digital LDR boards: LOW = bright (day) 

SOUND_ACTIVE_HIGH = True          # used only for DO fallback 

  

# Pulls (avoid fighting module comparators) 

LDR_PULL   = None                 # None => PUD_OFF 

SOUND_PULL = None                 # None => PUD_OFF (DO fallback only) 

  

# LED strip (WS2812/WS2811 on GPIO18/PWM) 

NUM_PIXELS   = 8 

LED_COLOR    = (0, 255, 80) 

LED_PIN      = 18                 # GPIO18 / PWM0 (Pin 12) 

BRIGHT_DAY   = 0.20 

BRIGHT_NIGHT = 0.50 

ON_HOLD_SECONDS = 2.0 

LOOP_SLEEP      = 0.05 

  

# ToF (legacy quick gate you had) 

HUMAN_DIST_MM   = 200 

HUMAN_WIN       = 5 

HUMAN_MIN_HITS  = 3 

  

# >>> NEW: Human-only detector thresholds (TOF + PIR) <<< 

# these work alongside your existing variables; feel free to tweak 

HUMAN_MIN_DIST_MM = 10       # ignore very near reflections 

HUMAN_MAX_DIST_MM = 2500      # ignore far background 

HUMAN_FRAMES_REQ  = 3        # consecutive frames within band 

HUMAN_MIN_SPEED   = 0.02      # m/s (reject static/clutter) 

HUMAN_MAX_SPEED   = 3.00      # m/s (reject spikes) 

REQUIRE_PIR_FOR_HUMAN = False # <-- allow TOF-only if PIR doesn’t fire 

# ------------------------------------------------------- 

  

# Sound (ANALOG via PCF8591) 

USE_PCF8591_SOUND = True 

PCF8591_ADDR   = 0x48 

SOUND_ADC_CH   = 0                # AO wired to AIN0 

SOUND_LATCH_SEC = 0.25            # hold after spike 

SND_ALPHA      = 0.02             # baseline smoothing (0.01..0.05) 

SND_BOOST      = 30               # baseline + this -> trigger (lower = more sensitive) 

SOUND_THRESH   = 80              # absolute floor (lower = more sensitive) 

  

# --- ADDED for analog LDR --- 

USE_PCF8591_LDR = True            # enable analog LDR reading 

LDR_ADC_CH      = 1               # LDR AO -> PCF8591 A1 

INVERT_LDR      = True            # many LDRs: higher raw in dark; True makes % = brighter 

LDR_SMOOTH_N    = 10              # moving average window 

PIN_LIGHT_PCT   = 1               # Blynk V1 for brightness percentage 

# Light color override based on darkness (using %) 

LDR_WHITE_THR = 40.0              # % below which LEDs turn all white (dark environment) 

LDR_SAMPLE_EVERY = 0.20           # seconds between actual I2C reads (cache) 

# Use analog LDR % to decide day/night (with hysteresis) 

USE_ANALOG_DAYNIGHT = True 

DAY_HIGH_THR_PCT  = 35.0   # > 35% => Day 

NIGHT_LOW_THR_PCT = 25.0   # < 25% => Night 

# --- END ADDED --- 

  

# OFF watchdog after turning OFF (helps if first LED misses a latch) 

OFF_WATCHDOG_REPEATS = 6          # ~1.2s of re-sent OFFs 

  

# Power/Energy model (approx for WS2812 @5V) 

LED_SUPPLY_VOLTS = 5.0 

  

# Blynk pins 

PIN_POWER_W   = 6                 # V6 (Watts) 

PIN_ENERGY_WH = 7                 # V7 (Wh, "power_wh") 

  

# >>> METRICS ADDED: pins for Pro widgets 

PIN_ACTIVITY   = 0   # V0  (0/1) Heatmap source (triggered) 

PIN_LIGHTSTATE = 4   # V4  (0/1) Light state 

PIN_USES       = 19  # V10 (int) Uses today 

PIN_ONTIME_S   = 11  # V11 (sec) On-time today 

PIN_SAVINGS_P  = 18  # V18 (%)   Savings vs baseline 

  

# LDR stability: day/night must remain the same for this long to be accepted 

LDR_STABLE_SEC = 3.0 

  

# --- ADDED (savings) --- 

# Baseline model: each "use" would have been 10 minutes of full-bright WHITE. 

SAVINGS_BASELINE_MIN_PER_USE = 10.0 

BASELINE_COLOR = (255, 255, 255)  # full white for worst-case power 

BASELINE_BRIGHTNESS = 1.0         # full brightness 

  

# Blynk pins for savings/baseline 

PIN_ENERGY_TODAY_WH   = 20  # V20: today's energy (actual) 

PIN_BASELINE_TODAY_WH = 21  # V21: today's baseline Wh (counterfactual) 

PIN_SAVINGS_TODAY_WH  = 22  # V22: today's savings Wh (= baseline - actual) 

PIN_SAVINGS_ARCHIVE   = 23  # V23: one point per day with the *final* savings of the day 

# --- END ADDED --- 

  

# --- COST SETTINGS --- 

COST_PER_KWH_NZD = 0.3567   # 35.67 cents per kWh in NZ 

PIN_SAVINGS_COST = 24       # V24 = Cost savings today (NZD) 

# --------------------- 

  

# ---------------- IMPORTS ---------------- 

import RPi.GPIO as GPIO 

import busio, board, adafruit_vl53l1x 

from rpi_ws281x import PixelStrip, Color, ws 

import BlynkLib 

  

# ----------- PCF8591 (multiple API compat) ----------- 

ADC = None 

_adc_obj = None 

_pcf_mode = None  # "module", "class", "smbus", or None 

try: 

    import PCF8591 as ADC  # your case 

    _pcf_mode = "module" 

    try: 

        if hasattr(ADC, "setup"): 

            ADC.setup(PCF8591_ADDR) 

    except Exception: 

        pass 

except Exception: 

    try: 

        from PCF8591 import PCF8591 as PCF 

        _adc_obj = PCF(PCF8591_ADDR) 

        _pcf_mode = "class" 

    except Exception: 

        try: 

            from smbus2 import SMBus 

            _adc_obj = SMBus(1) 

            _pcf_mode = "smbus" 

        except Exception: 

            _pcf_mode = None 

  

def pcf8591_read(ch: int) -> int: 

    """Return 0..255; raise if ADC unavailable.""" 

    if _pcf_mode == "module": 

        for attr in ("read", "readADC", "analogRead"): 

            if hasattr(ADC, attr): 

                try: 

                    v = getattr(ADC, attr)(ch) 

                    return int(v) & 0xFF 

                except Exception: 

                    pass 

        raise RuntimeError("PCF8591 module found but no usable read()") 

    elif _pcf_mode == "class": 

        for attr in ("read", "readADC", "analogRead"): 

            if hasattr(_adc_obj, attr): 

                try: 

                    v = getattr(_adc_obj, attr)(ch) 

                    return int(v) & 0xFF 

                except Exception: 

                    pass 

        raise RuntimeError("PCF8591 class instance has no usable read()") 

    elif _pcf_mode == "smbus": 

        ctrl = 0x40 | (ch & 0x03) 

        _adc_obj.write_byte(PCF8591_ADDR, ctrl) 

        _ = _adc_obj.read_byte(PCF8591_ADDR)     # dummy 

        _adc_obj.write_byte(PCF8591_ADDR, ctrl) 

        val = _adc_obj.read_byte(PCF8591_ADDR) 

        return int(val) & 0xFF 

    else: 

        raise RuntimeError("PCF8591 not available") 

  

# ---------------- GPIO SETUP ---------------- 

GPIO.setmode(GPIO.BCM) 

GPIO.setup(PIR_PIN, GPIO.IN, pull_up_down=(GPIO.PUD_DOWN if PIR_ACTIVE_HIGH else GPIO.PUD_UP)) 

GPIO.setup(LDR_PIN, GPIO.IN, pull_up_down=(GPIO.PUD_OFF if LDR_PULL is None else GPIO.PUD_OFF)) 

GPIO.setup(SOUND_PIN, GPIO.IN, pull_up_down=(GPIO.PUD_OFF if SOUND_PULL is None else GPIO.PUD_OFF)) 

  

def pir_now() -> int: 

    v = GPIO.input(PIR_PIN) 

    return 1 if ((v == GPIO.HIGH) if PIR_ACTIVE_HIGH else (v == GPIO.LOW)) else 0 

  

def ldr_is_day() -> bool: 

    v = GPIO.input(LDR_PIN) 

    return (v == GPIO.HIGH) if LDR_BRIGHT_HIGH else (v == GPIO.LOW) 

  

# ---- Sound: analog (preferred) + DO fallback edge latch ---- 

_sound_event_until = 0.0 

_last_snd_level = None 

SND_BASELINE = 0.0 

  

def sound_detected(now: float) -> int: 

    global _sound_event_until, _last_snd_level, SND_BASELINE 

    if USE_PCF8591_SOUND and (_pcf_mode is not None): 

        try: 

            v = pcf8591_read(SOUND_ADC_CH)  # 0..255 

            if SND_BASELINE == 0.0: 

                SND_BASELINE = float(v) 

            SND_BASELINE = (1.0 - SND_ALPHA) * SND_BASELINE + SND_ALPHA * float(v) 

            trig = v >= max(SND_BASELINE + SND_BOOST, SOUND_THRESH) 

            if trig: 

                _sound_event_until = now + SOUND_LATCH_SEC 

            return 1 if now < _sound_event_until else 0 

        except Exception: 

            pass 

    lvl = GPIO.input(SOUND_PIN) 

    cur = 1 if ((lvl == GPIO.HIGH) if SOUND_ACTIVE_HIGH else (lvl == GPIO.LOW)) else 0 

    if _last_snd_level is None: 

        _last_snd_level = cur 

    elif cur != _last_snd_level: 

        _sound_event_until = now + SOUND_LATCH_SEC 

        _last_snd_level = cur 

    return 1 if now < _sound_event_until else 0 

  

# --- ADDED for analog LDR (cached reads + % smoothing) --- 

_ldr_window = collections.deque(maxlen=LDR_SMOOTH_N) 

_last_ldr_read_t = 0.0 

_last_ldr_raw = None 

_last_ldr_pct = None 

  

def ldr_raw_and_percent(): 

    """ 

    Cached read: only touch I2C every LDR_SAMPLE_EVERY seconds. 

    Returns (raw_0_255, pct_0_100) or (None, None) if unavailable. 

    """ 

    global _last_ldr_read_t, _last_ldr_raw, _last_ldr_pct 

    if not (USE_PCF8591_LDR and (_pcf_mode is not None)): 

        return None, None 

  

    now = time.time() 

    if (now - _last_ldr_read_t) < LDR_SAMPLE_EVERY and _last_ldr_raw is not None: 

        return _last_ldr_raw, _last_ldr_pct 

  

    try: 

        raw = int(pcf8591_read(LDR_ADC_CH)) & 0xFF 

    except Exception: 

        return _last_ldr_raw, _last_ldr_pct  # keep last good value 

  

    adj = (255 - raw) if INVERT_LDR else raw 

    pct = (adj / 255.0) * 100.0 

    _ldr_window.append(pct) 

    smoothed = sum(_ldr_window) / len(_ldr_window) 

  

    _last_ldr_read_t = now 

    _last_ldr_raw = raw 

    _last_ldr_pct = smoothed 

    return raw, smoothed 

# --- END ADDED --- 

  

# ---------------- VL53L1X (ToF) ---------------- 

import busio, board, adafruit_vl53l1x 

i2c = busio.I2C(board.SCL, board.SDA) 

tof = adafruit_vl53l1x.VL53L1X(i2c) 

tof.start_ranging() 

def dist_mm() -> int: 

    d = tof.distance 

    return int(d) if d is not None else -1 

  

# ---------------- LED STRIP ---------------- 

from rpi_ws281x import PixelStrip, Color, ws 

STRIP_TYPE = ws.WS2811_STRIP_GRB 

strip = PixelStrip(NUM_PIXELS, LED_PIN, 800000, 10, False, 255, 0, STRIP_TYPE) 

strip.begin() 

  

def _color_scaled(rgb: tuple, b: float) -> int: 

    r = int(rgb[0] * b); g = int(rgb[1] * b); bl = int(rgb[2] * b) 

    return Color(r, g, bl) 

  

def leds_on(brightness: float, color: tuple = None): 

    """Turn on all LEDs with given brightness and optional color override.""" 

    strip.setBrightness(255) 

    c_rgb = color if color is not None else LED_COLOR 

    c = _color_scaled(c_rgb, brightness) 

    for i in range(NUM_PIXELS): 

        strip.setPixelColor(i, c) 

    strip.show() 

  

def leds_off_hard(): 

    strip.setBrightness(0); strip.show(); time.sleep(0.03) 

    for _ in range(2): 

        for i in range(NUM_PIXELS): strip.setPixelColor(i, Color(0,0,0)) 

        strip.show(); time.sleep(0.06) 

  

def leds_off_watchdog(): 

    for _ in range(OFF_WATCHDOG_REPEATS): 

        leds_off_hard() 

        time.sleep(0.2) 

  

# ---- GUARANTEED OFF ON EXIT ---- 

def destroy(): 

    try: 

        leds_off_watchdog() 

    finally: 

        try: 

            GPIO.setmode(GPIO.BCM) 

            GPIO.setup(LED_PIN, GPIO.OUT, initial=GPIO.LOW) 

        except Exception: 

            pass 

  

atexit.register(destroy) 

signal.signal(signal.SIGINT,  lambda s,f: (destroy(), sys.exit(0))) 

signal.signal(signal.SIGTERM, lambda s,f: (destroy(), sys.exit(0))) 

  

# ---------------- BLYNK (quiet) ---------------- 

import BlynkLib 

blynk = BlynkLib.Blynk(BLYNK_AUTH, server="blynk.cloud", port=80, log=lambda *a, **k: None) 

  

def led_current_amps(n_leds: int, brightness: float, color: tuple) -> float: 

    r,g,b = color 

    return n_leds * 0.02 * brightness * ((r + g + b) / 255.0) 

  

def inst_power_w(led_on: bool, brightness: float) -> float: 

    return 0.0 if not led_on else LED_SUPPLY_VOLTS * led_current_amps(NUM_PIXELS, brightness, LED_COLOR) 

  

# >>> METRICS ADDED: Blynk Pro metrics (KPI / Timeseries / Heatmap / Table) 

import datetime 

def _nz_now(): 

    return datetime.datetime.now()  # ensure system TZ is set to Pacific/Auckland 

  

class Metrics: 

    def __init__(self, blynk): 

        self.blynk = blynk 

        self.day_key = _nz_now().strftime("%Y-%m-%d") 

        self.energy_today_wh = 0.0 

        self.baseline_today_wh = 0.0 

        self.uses_today = 0 

        self.on_time_today_s = 0.0 

        self.prev_led_on = 0 

        self.last_sample_t = time.time() 

        self.baseline_per_use_wh = 0.0 

  

    def _maybe_roll_day(self, now_t): 

        today_key = _nz_now().strftime("%Y-%m-%d") 

        if today_key != self.day_key: 

            # --- archive yesterday's savings to V23 before reset --- 

            try: 

                baseline_wh = self.uses_today * self.baseline_per_use_wh 

                savings_wh  = max(0.0, baseline_wh - self.energy_today_wh) 

                self.blynk.virtual_write(PIN_SAVINGS_ARCHIVE, round(savings_wh, 3))  # V23 single daily point 

            except: 

                pass 

            # ------------------------------------------------------ 

  

            try: 

                self.blynk.virtual_write(PIN_USES, 0) 

                self.blynk.virtual_write(PIN_ONTIME_S, 0) 

                self.blynk.virtual_write(PIN_ENERGY_WH, 0.0) 

                self.blynk.virtual_write(PIN_SAVINGS_P, 0.0) 

                # savings resets 

                self.blynk.virtual_write(PIN_BASELINE_TODAY_WH, 0.0)  # V21 

                self.blynk.virtual_write(PIN_SAVINGS_TODAY_WH, 0.0)   # V22 

                self.blynk.virtual_write(PIN_SAVINGS_COST, 0.0)       # V24 cost NZD 

            except: 

                pass 

  

            self.day_key = today_key 

            self.energy_today_wh = 0.0 

            self.baseline_today_wh = 0.0 

            self.uses_today = 0 

            self.on_time_today_s = 0.0 

  

    def update(self, *, now_t: float, led_on: int, triggered: int, 

               power_w: float, night_brightness: float): 

        self._maybe_roll_day(now_t) 

        # Heatmap + Light state + Power 

        try: 

            self.blynk.virtual_write(PIN_ACTIVITY, int(triggered))   # V0 

            self.blynk.virtual_write(PIN_LIGHTSTATE, int(led_on))    # V4 

            self.blynk.virtual_write(PIN_POWER_W, round(power_w, 3)) # V6 

        except: 

            pass 

  

        # Integration 

        dt = max(0.0, now_t - self.last_sample_t) 

        self.last_sample_t = now_t 

        self.energy_today_wh += (power_w * dt) / 3600.0 

        p_baseline = LED_SUPPLY_VOLTS * led_current_amps(NUM_PIXELS, BRIGHT_NIGHT, LED_COLOR) 

        self.baseline_today_wh += (p_baseline * dt) / 3600.0 

        if led_on: 

            self.on_time_today_s += dt 

  

        # Uses on rising edge 

        if (not self.prev_led_on) and led_on: 

            self.uses_today += 1 

            try: 

                self.blynk.virtual_write(PIN_USES, self.uses_today) 

            except: 

                pass 

        self.prev_led_on = 1 if led_on else 0 

  

        # Publish energy & on-time 

        try: 

            self.blynk.virtual_write(PIN_ENERGY_WH, round(self.energy_today_wh, 3))  # V7 

            self.blynk.virtual_write(PIN_ONTIME_S, int(self.on_time_today_s))        # V11 

        except: 

            pass 

  

        # Savings % vs continuous-night baseline (legacy) 

        savings_wh_legacy = max(0.0, self.baseline_today_wh - self.energy_today_wh) 

        savings_pct = (savings_wh_legacy / self.baseline_today_wh * 100.0) if self.baseline_today_wh > 1e-9 else 0.0 

        try: 

            self.blynk.virtual_write(PIN_SAVINGS_P, round(savings_pct, 2))           # V18 

        except: 

            pass 

  

        # --- ADDED (savings): per-use baseline (10 min @ full-white) + daily savings & cost --- 

        # Compute per-use baseline once 

        if self.baseline_per_use_wh == 0.0: 

            p_full_white_w = LED_SUPPLY_VOLTS * led_current_amps( 

                NUM_PIXELS, BASELINE_BRIGHTNESS, BASELINE_COLOR 

            ) 

            self.baseline_per_use_wh = (p_full_white_w * (SAVINGS_BASELINE_MIN_PER_USE * 60.0)) / 3600.0  # Wh/use 

  

        # Compute EVERY loop 

        baseline_today_wh = self.uses_today * self.baseline_per_use_wh 

        savings_today_wh  = max(0.0, baseline_today_wh - self.energy_today_wh) 

  

        # Cost (NZD) — 4 dp so tiny values don't round to zero 

        savings_cost_nzd = (savings_today_wh / 1000.0) * COST_PER_KWH_NZD 

        COST_DECIMALS = 4 

  

        try: 

            self.blynk.virtual_write(PIN_BASELINE_TODAY_WH, round(baseline_today_wh, 3))    # V21 

            self.blynk.virtual_write(PIN_SAVINGS_TODAY_WH,  round(savings_today_wh,  3))    # V22 

            self.blynk.virtual_write(PIN_SAVINGS_COST,      round(savings_cost_nzd, COST_DECIMALS))  # V24 

        except: 

            pass 

        # --- END ADDED --- 

  

# ---------------- MAIN ---------------- 

shutdown = False 

signal.signal(signal.SIGINT,  lambda *_: globals().__setitem__('shutdown', True)) 

signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__('shutdown', True)) 

  

def main(): 

    print("Stairway — GPIO18/PWM, GRB + PCF8591 sound. Ctrl+C to stop.") 

    leds_on(0.25); time.sleep(0.3); leds_off_hard() 

  

    led_on = False 

    cur_brightness = BRIGHT_DAY 

    last_trigger_ts = 0.0 

  

    total_energy_wh = 0.0 

    last_energy_t = time.time() 

    last_blynk_push = 0.0 

    last_print = 0.0 

    last_v0 = -1 

    # Analog day/night state 

    analog_day = True 

  

    # --- ADDED: false-trigger + day roll helpers --- 

    false_triggers_today = 0 

    prev_led_on_state = 0 

    day_key = time.strftime("%Y-%m-%d") 

    false_active = False          # currently in a false-trigger episode? 

    false_arm_t  = 0.0            # when we first saw LED on with no sensors 

    FALSE_DEBOUNCE = 0.30         # seconds LED must stay on with no sensors to count 

  

    # ---------------------------------------------- 

  

    # --- LDR stability (init) --- 

    stable_day = ldr_is_day() 

    last_day_change_ts = time.time() 

    # ----------------------------- 

  

    # ToF smoothing 

    dwin = collections.deque(maxlen=HUMAN_WIN) 

  

    # >>> NEW: human filter state 

    band_run = 0 

    prev_m = None 

    prev_t = None 

    # ---------------------------- 

  

    # Initial Blynk values 

    for pin,val in ((PIN_POWER_W,0.0),(PIN_ENERGY_WH,0.0),(0,0)): 

        try: blynk.virtual_write(pin, val) 

        except: pass 

  

    try: 

        blynk.virtual_write(PIN_SAVINGS_COST, 0.0)  # V24 cost NZD 

    except: 

        pass 

  

    # --- ADDED (savings init) --- 

    try: 

        blynk.virtual_write(PIN_BASELINE_TODAY_WH, 0.0)  # V21 

        blynk.virtual_write(PIN_SAVINGS_TODAY_WH,  0.0)  # V22 

        # V23 archive happens at rollover; no init needed 

    except: 

        pass 

    # --- END ADDED --- 

  

    # --- ADDED: init V19 (energy today) and V8 (false triggers) --- 

    try: 

        blynk.virtual_write(20, 0)  # V19 = Energy today (Wh) 

        blynk.virtual_write(8,  0)  # V8  = False triggers today (count) 

    except: 

        pass 

    # -------------------------------------------------------------- 

  

    # --- SOUND init (V3 raw) --- 

    try: 

        blynk.virtual_write(3, 0)  # V3 sound raw 

    except: 

        pass 

    # ------------------------------------- 

  

    # TOF init to V2 

    try: 

        blynk.virtual_write(2, 0)   # V2 = TOF (mm) initial 

    except: 

        pass 

  

    off_watchdog_until = 0.0 

  

    # >>> METRICS ADDED: initialize metrics 

    metrics = Metrics(blynk) 

    # ----------------------------------- 

  

    while not shutdown: 

        try: blynk.run() 

        except: pass 

  

        now = time.time() 

  

        # --------- Sensors (TOF + PIR + SOUND + LDR) ---------- 

        d = dist_mm()                             # mm; keep exactly as you had 

        d_eff = d if (d is not None and d > 0) else 9999 

        dwin.append(d_eff) 

  

        motion = pir_now() 

        day    = ldr_is_day() 

        snd    = sound_detected(now) 

  

        # ---------- NEW: human-only detector (PIR + TOF) ---------- 

        # Median TOF (m) from your sliding window 

        if len(dwin): 

            m_mm = sorted(dwin)[len(dwin)//2] 

        else: 

            m_mm = 9999 

        m = m_mm / 1000.0  # meters 

  

        # consecutive frames inside band 

        if HUMAN_MIN_DIST_MM <= m_mm <= HUMAN_MAX_DIST_MM: 

            band_run += 1 

        else: 

            band_run = 0 

  

        # speed gate (m/s) using previous median 

        speed_ok = True 

        if (prev_m is not None) and (prev_t is not None): 

            dt = max(1e-3, now - prev_t) 

            v = abs(m - prev_m) / dt 

            speed_ok = (HUMAN_MIN_SPEED <= v <= HUMAN_MAX_SPEED) 

  

        # decide via TOF 

        human_by_tof = (band_run >= HUMAN_FRAMES_REQ) and speed_ok 

  

        # fuse with PIR (optional requirement) 

        if REQUIRE_PIR_FOR_HUMAN: 

            human = 1 if (motion == 1 and human_by_tof) else 0 

        else: 

            # either PIR OR solid TOF qualifies 

            human = 1 if (motion == 1 or human_by_tof) else 0 

  

        # (optional) quick debug: uncomment to see gates 

        # print(f"[det] PIR={motion} m={m:.2f}m band_run={band_run} speed_ok={speed_ok} -> human={human}") 

  

        prev_m, prev_t = m, now 

# --------------- END human-only detector ----------------- 

  

        # --- NEW: false trigger = LED ON while no sensors active (debounced) --- 

        no_sensors = (human == 0) and (motion == 0) and (snd == 0) 

  

        if led_on and no_sensors: 

            if not false_active: 

                false_active = True 

                false_arm_t = now 

            else: 

                if (now - false_arm_t) >= FALSE_DEBOUNCE: 

                    false_triggers_today += 1 

                    try: 

                        blynk.virtual_write(8, false_triggers_today)  # V8 

                    except: 

                        pass 

        else: 

            false_active = False 

        # --- END NEW --- 

  

        # --- LDR stability (debounce/hysteresis) --- 

        if day != stable_day: 

            if (now - last_day_change_ts) >= LDR_STABLE_SEC: 

                stable_day = day 

                last_day_change_ts = now 

        else: 

            last_day_change_ts = now 

        # ------------------------------------------- 

  

        # --- ADDED for analog LDR --- 

        ldr_raw, ldr_pct = ldr_raw_and_percent()  # safe if ADC missing 

        if USE_ANALOG_DAYNIGHT and (ldr_pct is not None): 

            if analog_day and (ldr_pct < NIGHT_LOW_THR_PCT): 

                analog_day = False 

            elif (not analog_day) and (ldr_pct > DAY_HIGH_THR_PCT): 

                analog_day = True 

            day = analog_day 

        # --- END ADDED --- 

  

        triggered = 1 if (human or motion or snd) else 0 

  

        # Status (2x/sec) 

        if now - last_print >= 0.5: 

            p_now = inst_power_w(led_on, cur_brightness) 

            print(("Day" if day else "Night"), 

                  "| human",human,"motion",motion,"sound",snd, 

                  "| dist", (d if d>0 else "N/A"), 

                  "| LED",("ON" if led_on else "OFF"), 

                  "| P(W)", round(p_now,3), 

                  "| E(Wh)", round(total_energy_wh,3), 

                  "| uses:", metrics.uses_today) 

            last_print = now 

  

        # Optional V0 mirror 

        if triggered != last_v0: 

            try: blynk.virtual_write(0, triggered) 

            except: pass 

            last_v0 = triggered 

  

        # Brightness by ambient (use stable day/night) 

        target_b = BRIGHT_DAY if stable_day else BRIGHT_NIGHT 

  

        if triggered: 

            last_trigger_ts = now 

  

            # choose LED colour by light intensity % 

            if ldr_pct is not None and ldr_pct < LDR_WHITE_THR: 

                led_color_now = (255, 255, 255)    # all white 

            else: 

                led_color_now = LED_COLOR          # normal colour 

  

            if (not led_on) or abs(target_b - cur_brightness) > 1e-6: 

                cur_brightness = target_b 

                leds_on(cur_brightness, color=led_color_now) 

                led_on = True 

  

            off_watchdog_until = 0.0 

  

        else: 

            if led_on and (now - last_trigger_ts) >= ON_HOLD_SECONDS: 

                leds_off_hard() 

                led_on = False 

                off_watchdog_until = now + (OFF_WATCHDOG_REPEATS * 0.2) 

  

        # OFF watchdog active? 

        if (not led_on) and (off_watchdog_until > now): 

            leds_off_hard() 

  

        # remember LED state for next loop (for false-trigger logic) 

        prev_led_on_state = 1 if led_on else 0 

  

        # Energy integration (legacy local running total kept intact) 

        dt = now - last_energy_t 

        if dt > 0: 

            p_now = inst_power_w(led_on, cur_brightness) 

            total_energy_wh += (p_now * dt) / 3600.0 

            last_energy_t = now 

  

        # Blynk push (1 Hz) — keep your original numeric channels 

        if now - last_blynk_push >= 1.0: 

            try: 

                blynk.virtual_write(PIN_POWER_W, round(inst_power_w(led_on, cur_brightness), 3)) 

                blynk.virtual_write(PIN_ENERGY_WH, round(total_energy_wh, 3)) 

                blynk.virtual_write(PIN_ENERGY_TODAY_WH, round(metrics.energy_today_wh, 3)) 

  

                try: 

                    baseline_today_wh = metrics.uses_today * metrics.baseline_per_use_wh 

                    savings_today_wh  = max(0.0, baseline_today_wh - metrics.energy_today_wh) 

                    savings_cost_nzd  = (savings_today_wh / 1000.0) * COST_PER_KWH_NZD 

                    blynk.virtual_write(PIN_SAVINGS_COST, round(savings_cost_nzd, 4))  # V24 

                except: 

                    pass 

                last_blynk_push = now 

  

                # SOUND publish (V3 raw) 

                try: 

                    if USE_PCF8591_SOUND and (_pcf_mode is not None): 

                        sound_raw = int(pcf8591_read(SOUND_ADC_CH)) & 0xFF   # 0..255 

                    else: 

                        sound_raw = 255 if snd else 0 

                except: 

                    sound_raw = 0 

                blynk.virtual_write(3, sound_raw)  # V3 raw level 

  

                # TOF to V2 (send 0 when invalid) 

                blynk.virtual_write(2, int(d) if (d is not None and d > 0) else 0) 

  

                # analog LDR % 

                if ldr_pct is not None: 

                    blynk.virtual_write(PIN_LIGHT_PCT, round(ldr_pct, 1))  # V1 

            except: 

                pass 

            last_blynk_push = now 

  

        # >>> METRICS ADDED: feed Pro widgets every loop 

        metrics.update( 

            now_t=now, 

            led_on=1 if led_on else 0, 

            triggered=1 if triggered else 0, 

            power_w=inst_power_w(led_on, cur_brightness), 

            night_brightness=BRIGHT_NIGHT 

        ) 

  

        # daily roll-over for V19/V8 at local midnight 

        current_day_key = time.strftime("%Y-%m-%d") 

        if current_day_key != day_key: 

            day_key = current_day_key 

            false_triggers_today = 0 

            try: 

                blynk.virtual_write(8, 0)   # reset false triggers 

                blynk.virtual_write(20, 0)  # reset energy today 

            except: 

                pass 

  

        time.sleep(LOOP_SLEEP) 

  

    # Graceful shutdown 

    try: 

        tof.stop_ranging() 

    except: 

        pass 

    destroy() 

    try: 

        GPIO.cleanup() 

    except: 

        pass 

    print("Stopped. Energy Wh:", round(total_energy_wh, 3)) 

  

if __name__ == "__main__": 

    main() 
