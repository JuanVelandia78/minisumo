---
name: CAR-LOSS Consola
description: Live control console for the CAR-LOSS minisumo robot; the console is the dohyō.
colors:
  ring: "#0e0e0f"
  stage: "#111112"
  shell: "#1a1613"
  shell-2: "#221c18"
  shell-3: "#2a231e"
  rule: "#3a312b"
  rule-soft: "#2c2520"
  tawara: "#ece6dc"
  muted: "#b5a99c"
  faint: "#958a7e"
  ink-on-tawara: "#111111"
  umber: "#8c5b3b"
  umber-hi: "#c48e66"
  car: "#f2c21b"
  fusa: "#56c4a3"
  ok: "#7fd4a1"
  amber: "#e9a53f"
  red: "#ff5d55"
  stop: "#b8232c"
  stop-hi: "#d22f38"
typography:
  display:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "70px"
    fontWeight: 600
    lineHeight: 1
    letterSpacing: "-0.01em"
    fontFeature: "tnum"
  readout:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "40px"
    fontWeight: 600
    lineHeight: 1
    fontFeature: "tnum"
  headline:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "36px"
    fontWeight: 600
    lineHeight: 1
    fontFeature: "tnum"
  wordmark:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "21px"
    fontWeight: 700
    lineHeight: 1
    letterSpacing: "0.06em"
  title:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "20px"
    fontWeight: 600
    lineHeight: 1.15
    letterSpacing: "0.03em"
  plot-title:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "16px"
    fontWeight: 600
    letterSpacing: "0.03em"
  unit:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "15px"
    fontWeight: 500
    letterSpacing: "0.02em"
  data:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "13px"
    fontWeight: 500
    letterSpacing: "0.02em"
    fontFeature: "tnum"
  control:
    fontFamily: "Barlow Semi Condensed, Barlow, system-ui, sans-serif"
    fontSize: "13px"
    fontWeight: 600
    lineHeight: 1
    letterSpacing: "0.06em"
  body:
    fontFamily: "Barlow, system-ui, sans-serif"
    fontSize: "14px"
    fontWeight: 400
    lineHeight: 1.4
  label:
    fontFamily: "Barlow, system-ui, sans-serif"
    fontSize: "11px"
    fontWeight: 500
    lineHeight: 1.2
    letterSpacing: "0.11em"
rounded:
  hairline: "1px"
  sm: "2px"
  round: "50%"
spacing:
  gap-sm: "8px"
  gap-md: "12px"
  gap-lg: "16px"
  module: "14px 18px 15px"
  panel: "18px 20px 20px"
  view: "22px 26px 30px"
  bar-h: "52px"
  rail-w: "212px"
  tel-w: "392px"
components:
  link-pill:
    backgroundColor: "transparent"
    textColor: "{colors.muted}"
    typography: "{typography.data}"
    rounded: "{rounded.sm}"
    padding: "7px 11px"
    height: "30px"
  stop-button:
    backgroundColor: "{colors.stop}"
    textColor: "#ffffff"
    rounded: "{rounded.sm}"
    padding: "0 14px 0 16px"
    height: "38px"
  stop-button-hover:
    backgroundColor: "{colors.stop-hi}"
  stop-button-engaged:
    backgroundColor: "{colors.tawara}"
    textColor: "{colors.ink-on-tawara}"
  rail-tab:
    backgroundColor: "transparent"
    textColor: "{colors.muted}"
    padding: "11px 18px"
  rail-tab-hover:
    backgroundColor: "{colors.shell-2}"
    textColor: "{colors.tawara}"
  rail-tab-selected:
    backgroundColor: "{colors.shell-3}"
    textColor: "{colors.tawara}"
  telemetry-module:
    backgroundColor: "{colors.shell}"
    textColor: "{colors.tawara}"
    padding: "{spacing.module}"
  meter:
    backgroundColor: "{colors.rule-soft}"
    height: "6px"
  plot:
    backgroundColor: "{colors.stage}"
    padding: "14px 16px 10px"
  button:
    backgroundColor: "{colors.shell-2}"
    textColor: "{colors.tawara}"
    typography: "{typography.control}"
    rounded: "{rounded.sm}"
    padding: "0 13px"
    height: "34px"
  button-hover:
    backgroundColor: "{colors.shell-3}"
  button-primary:
    backgroundColor: "{colors.tawara}"
    textColor: "{colors.ink-on-tawara}"
    rounded: "{rounded.sm}"
    height: "34px"
  segmented-selected:
    backgroundColor: "{colors.tawara}"
    textColor: "{colors.ink-on-tawara}"
    padding: "7px 13px"
  gains-input:
    backgroundColor: "{colors.ring}"
    textColor: "{colors.tawara}"
    rounded: "{rounded.sm}"
    height: "32px"
    padding: "0 8px"
  gains-input-dirty:
    textColor: "{colors.car}"
  drive-pad:
    backgroundColor: "{colors.ring}"
    rounded: "{rounded.round}"
    size: "210px"
  drive-pad-knob:
    backgroundColor: "{colors.car}"
    rounded: "{rounded.round}"
    size: "40px"
---

# Design System: CAR-LOSS Consola

## Overview

**Creative North Star: "The Dohyō"**

The console is the minisumo ring itself. A matte black dohyō sits at the center of the screen with the team's own SolidWorks robot running on it. Its white tawara edge is the main instrument: the real and commanded heading ride on it, and it lights up where the robot nears the edge. Around the ring is a warm, dark clay shell (top bar, rail, telemetry column). That shell is cut into dense modules by 1px rules, never broken into floating cards. Numbers are large, condensed and tabular, and they are the loudest thing in every module.

Color is earned. Tawara white carries rules and real measured values. One yellow, the CAR-LOSS wheel color, is reserved for what the operator orders. Fusa green belongs to Kalman, umber to structure and to the ESP32 filter. Red belongs to stop and critical states. Everything else is warm gray on black, so any colored pixel means something.

Density is high and calm, closer to an instrument panel than a dashboard. There is no decorative entrance motion. What moves is the robot, the heading ticks and the camera, all damped. Missing or stale data is never shown as zero: it renders as an em dash or dims to faint.

**Key Characteristics:**
- The 3D dohyō fills the center stage; the console is built around the ring, not as a tile beside it.
- A warm clay shell, split into a mosaic of modules by 1px rules.
- Corners at most 2px. Full circles (the ring, the pad, the status dots) are native to the world.
- Condensed tabular numerals as the dominant type, with small tracked uppercase labels.
- One reserved ink (yellow) for operator intent; one series color (fusa) for Kalman.
- Missing data reads "—", never 0.

## Colors

A matte-black-and-clay palette lit by tawara white, with five semantic inks, each with a single job.

### Primary
- **Tawara White** (tawara): the ring's straw edge. Used for real measured values, active text, the selected tab and its dot, the real-heading tick, the raw-signal trace in plots (at 42% opacity), the meter's measured bar, the focus outline, and pressed or primary button fills (with ink-on-tawara text).
- **Ink on Tawara** (ink-on-tawara): the dark text on every tawara or white fill (primary button, pressed segment, engaged stop, toast, release button).

### Secondary
- **Wheel Yellow** (car): the CAR-LOSS wheel color. Used only for operator intent: the commanded-heading tick, the trajectory trail (80% opacity), the meter's reference mark, the glove order value and its source tag, the manual-drive knob and readout, the glove dial needle, gain fields edited but not yet applied, the "sent" flash on quick actions, command log lines, and the text caret.
- **Fusa Green** (fusa): the green of the dohyō's tassels. Kalman only: the Kalman trace in every plot (1.7px, the heaviest line), the "reducción" statistic, and the matching legend ink.
- **Shikiri Umber** (umber): the shikiri start lines. Used for secondary structure: the start lines on the 3D ring, the drive-pad axes, and the glove dial's dead-zone arc.
- **Kiln Umber** (umber-hi): the ESP32 firmware filter only. Its 1.3px trace in plots (optional), its legend ink, and the accent color of its checkbox.

### Tertiary (state)
- **Signal Red** (red): critical states only. Edge ≤6 cm, battery below 6.6 V, critical log lines.
- **Warning Amber** (amber): warning states. Edge ≤12 cm, battery below 7.0 V, a link that is late, warning log lines.
- **Healthy Green** (ok): healthy states. A live link dot, battery at 7.0 V or above, "ok" in the Sistema table.
- **Stop Red** (stop / stop-hi on hover): the PARAR button and the emergency-stop banner. It is never used for data.

### Neutral
- **Ring Black** (ring): the page and the dohyō surface. Also the fill of inputs and the drive pad.
- **Stage Black** (stage): the plot cells. The 3D stage adds a radial lift to #161517 at its center.
- **Clay Shell** (shell): the top bar, the rail, the telemetry column and the panels. **Clay 2** (shell-2) is the hover and default button fill. **Clay 3** (shell-3) is the selected tab and button hover.
- **Rule** (rule): every 1px structural rule, the segment dividers, the range track and the scrollbar thumb. **Soft Rule** (rule-soft) is used for inner list and table dividers and for the meter track.
- **Muted** (muted): labels, units, secondary text, idle controls.
- **Faint** (faint): axis text, timestamps, the idle link dot, stale values, the "off" state.

### Named Rules
**The Reserved Ink Rule.** Wheel Yellow means "the operator ordered this". If a pixel is yellow and nobody commanded it, it is wrong. Never use yellow for emphasis, links, warnings or brand decoration.

**The One Series Color Rule.** Signals are white hairlines. Only the Kalman estimate gets color (fusa), and only the ESP32 filter gets the second (umber-hi). The amber/red/green state inks never draw a signal series.

**The Two Reds Rule.** Stop red is the control you press; signal red is the reading you fear. Do not swap them.

## Typography

**Display / Numeral Font:** Barlow Semi Condensed 500/600/700 (falls back to Barlow, system-ui)
**Body Font:** Barlow 400/500/600 (falls back to system-ui)
**Mono:** ui-monospace, Consolas, used only for literal firmware command strings

**Character:** A condensed grotesk built for numerals and labels, set tabular, next to the plain humanist Barlow for prose. Both are self-hosted (woff2, `font-display: swap`) so the console works offline.

### Hierarchy
- **Display** (600, 70px, 1, −0.01em, tabular; 58px at ≤1280px): the single hero numeral, which is speed.
- **Readout** (600, 40–46px, 1, tabular): the edge-distance readout on the stage (30px on phone) and the glove dial values (46px).
- **Headline** (600, 36px, 1, tabular): module values (heading, turn rate, wheels, battery, order, manual drive).
- **Wordmark** (700, 21px, 0.06em): "CAR·LOSS", where the dot is a tawara ring.
- **Title** (600, 20px, 1.15, 0.03em): view and panel headings. Plot titles use 16px.
- **Unit** (500, 15px, muted): the unit set after a value at the baseline ("cm/s", "°", "V").
- **Data** (500, 13px, muted, with tabular tawara `b` values): the sub-lines under values, link pills, legends and plot stats.
- **Control** (600, 12–13px, 0.06–0.1em, uppercase): buttons and segmented controls. Rail tabs are 14px at 0.05em. The stop label is 700, 16px, 0.16em.
- **Body** (400, 14px, 1.4): prose. Hints cap at 68ch and footnotes at 90ch.
- **Label** (Barlow 500, 11px, 0.11em, uppercase, muted): module names, the "Al borde" readout label, table heads and the rail section head.

### Named Rules
**The Scale Contrast Rule.** Hierarchy comes from size, not color or weight: a 70/36/13/11 jump in one module. A value always outranks its label by at least 3x.

**The Tabular Rule.** Every live number uses tabular figures, a decimal comma and a typographic minus (−), so values do not jitter as they update.

## Layout

The desktop is a fixed app grid, not a page. It has a 52px top bar across the full width, a 212px left rail, and a main area. In "En vivo", the main area is the 3D stage plus a 392px telemetry column. An emergency-stop banner row appears under the bar only while the stop is engaged. Secondary views (Señales, Control, Guante, Sistema) scroll inside the main area with 22/26/30px padding.

The telemetry column is a two-column mosaic. Speed and the event log span both columns. Heading and turn rate, the two wheels, and battery and order pair up. Modules are separated by a 1px rule on their bottom edge and on the left edge of the right-hand cell. Secondary views reuse the same mosaic: plot, control, glove and system grids draw a rule on top and left of the grid, and each cell draws its right and bottom rules. Panels inside a mosaic lose their own border and radius.

Spacing is tight: gaps of 8/12/16px, module padding 14/18/15px, panel padding 18/20/20px.

Responsive:
- **≤1280px:** the rail collapses to 64px icon-only. The telemetry column narrows to 340px and the display numeral drops to 58px.
- **≤1100px:** plots, control and system grids go to one column, and the glove grid goes to two. The mode pill is hidden.
- **≤820px (phone):** one zone per screen. The bar becomes 56px and the rail becomes a 60px bottom tab bar with stacked icon and label. "En vivo" becomes a vertical scroll-snap of two full pages: the stage, then telemetry. In portrait, the Dojo camera rises to 72° elevation and the ring fills the height.
- **≤520px:** link pills drop to a status dot plus a ring icon (robot) or hand icon (glove). Their full state lives in an aria-label and title.

### Named Rules
**The Ruled Mosaic Rule.** Adjacent modules share exactly one 1px rule. Never gap them apart, never give them their own borders or radius, never float them as cards.

**The Ring Is the Stage Rule.** The dohyō takes the full center stage, sized so the base fills about 92% of the limiting dimension. It is never shrunk into a tile.

## Elevation & Depth

The interface is flat. Depth comes from tonal layering: ring black, then clay shell, then clay 2/3 on hover and selection. The only real depth is the 3D dohyō. It uses a warm hemisphere light, a key light, a warm rim light and a soft radial contact shadow under the robot. The stage's radial gradient (#161517 to ring) acts as a pool of light.

### Shadow Vocabulary
- **Knob lift** (`box-shadow: 0 4px 14px rgba(0,0,0,.5)`): only the yellow drive-pad knob, so it reads as a puck you grab.
- **Toast float** (`box-shadow: 0 8px 28px rgba(0,0,0,.45)`): only the transient tawara toast.
- **Tawara ring** (`box-shadow: 0 0 0 7px var(--tawara)`): not a shadow but a spread ring that draws the white edge around the drive pad.

### Named Rules
**The Flat Shell Rule.** Modules, panels, buttons and pills have no shadow. If something needs to stand out, give it tawara or a semantic ink, not elevation.

## Shapes

Corners are nearly square: 2px on pills, buttons, inputs, segments, panels, kbd keys, the toast and scrollbar thumbs, and 1px on the range thumb. Panels inside a mosaic go to 0. The exception is the circle, which is native to the world: the dohyō, the drive pad (a 210px disc ringed in 7px tawara), the pad knob, the status dots and the wordmark dot are full circles. Lines are 1px (rules), 1.25–1.7px (data traces) and 2px (meter reference, range track, pad axes, legend swatches). Icons are 20px line icons, stroke 1.5, with round caps and joins, drawn in currentColor.

### Named Rules
**The Two-Pixel Rule.** No rectangle rounds past 2px. Round things are full circles, never pills or soft cards.

## Components

### Link Pills
Status at a glance, top bar.
- **Shape:** 2px, a 1px rule border, at least 30px tall, padding 7px 11px.
- **Content:** a 7px status dot, a tracked uppercase tawara name ("ROBOT", "GUANTE"), then a muted rate and age line.
- **States:** the dot is ok green (live), amber (late), or hollow with a faint stroke (off). The mode pill uses a dashed border.
- **Phone:** at ≤820px the rate and age line drops. At ≤520px the name drops and a 15px ring or hand icon takes its place, and an aria-label carries the full state.

### Stop Button
The one control reachable from anywhere (Space).
- **Shape:** 2px, 38px tall (40px on phone), pushed to the far right of the bar.
- **Style:** stop red with white 700 16px uppercase text tracked 0.16em, a square stop icon and a white-outlined "Espacio" key.
- **Hover / Active:** stop-hi on hover, and a 1px push down when pressed (0.15s background, ease).
- **Engaged:** it inverts to tawara with ink text. A full-width stop-red banner appears below the bar with a white "Liberar" button.

### Rail Tabs
- **Style:** 20px icon and an uppercase 14px condensed label, muted, padding 11px 18px. The shortcut key fades in on hover.
- **Hover:** tawara text on clay 2. **Selected:** tawara text on clay 3, marked by a 5px tawara dot at the left edge. The selected tab has no colored bar.
- **Quick actions** (R, C, T) sit below a tracked label head. They flash yellow ("sent") when fired, because they are operator commands.
- **Responsive:** icon-only at ≤1280px. On phone it becomes a bottom bar with stacked icon and label, and the dot moves to the top center.

### Telemetry Modules
- **Structure:** a tracked label, then the value (display or headline numeral with a muted unit), then an optional data sub-line with tawara values. Speed adds a 44px history sparkline (a 1.25px tawara line over a 1px rule baseline, faint axis text).
- **States:** battery colors its value ok, amber or red. The order module's value is yellow. When robot telemetry goes stale, the whole column's values turn faint and the meters drop to 35% opacity.
- **Event log:** a 60px faint timestamp column with soft-rule dividers. Lines are colored by kind: amber warn, red crit, tawara good, yellow cmd.

### Meters
Measured against commanded, in one 6px bar.
- **Track:** soft rule, square ends.
- **Measured:** a tawara fill from the left.
- **Commanded:** a 2px × 14px yellow reference mark that overhangs the track.
- **Motion:** both move with a 0.12s ease transition.

### Plots
- **Cell:** stage black, inside the ruled mosaic. The header has a 16px title, a faint unit, and right-aligned stats (σ crudo, σ Kalman, reducción in fusa).
- **Traces:** raw is tawara 1px at 42%, the ESP32 filter is umber-hi 1.3px (optional), Kalman is fusa 1.7px.
- **Axes:** grid lines are rule at 70%, axis text is faint Barlow 11px, and the right end of the time axis reads "ahora".
- **Empty:** "Sin datos en esta ventana" is centered in muted text.
- **Glove plot:** separates pitch and roll with solid versus dashed lines, not with new colors.

### Gains Fields
- **Row:** a label with a faint sub-note, a range slider (2px rule track, 12×18px tawara thumb with a 1px radius), and a 32px right-aligned tabular number field on ring black.
- **Focus:** the border turns tawara.
- **Dirty:** an edited but unapplied value turns border and text yellow, because it is pending operator intent.
- **Actions:** a ghost "Valores del firmware" button and a tawara primary "Aplicar al robot" button.

### Buttons and Segments
- **Button:** 34px tall, 2px radius, on clay 2 with a rule border, 600 13px uppercase text. Clay 3 on hover.
- **Primary:** a tawara fill with ink text, white on hover. **Ghost:** transparent. **Pressed:** a tawara fill.
- **Segments** (camera presets, time window): one rule-bordered strip with 1px internal dividers. The selected segment is a tawara fill with ink text.

### Drive Pad
A miniature dohyō you can drive.
- **Shape and color:** a 210px ring-black disc with a 7px tawara edge and umber cross axes at 60% and 35% opacity. A 40px yellow knob carries the knob lift.
- **Behavior:** drag (grab cursor) or WASD. On release the knob springs back to center (0.2s ease), with no transition while it is held.
- **Readout:** beside the pad, a yellow headline value and a limit slider.

### Tawara Instrument (signature)
The ring's edge is the main gauge.
- **Heading ticks:** a white real-heading tick and a yellow commanded-heading tick ride just outside the tawara. Their angles interpolate with damping. A fixed heading scale sits on the base: ticks every 10°, longer at 30° and longest at 90°.
- **Edge arc:** a 60° arc centered on the bearing to the robot. It appears in steps: amber when the robot is 12 cm or less from the edge, red at 6 cm or less. While lit, it visibly narrows the tawara: only the outer 40% of the edge takes the color, and the inner 60% is cut to ring black.
- **Readout:** the stage's "Al borde" readout (40px, top right) colors its label and value the same amber or red.
- **Trail:** the trajectory is drawn in yellow, since it is the commanded path.

### Motion
- **Character:** damped and single-axis, with no decorative entrances.
- **Scene damping:** the robot pose and both ticks follow their targets exponentially (τ ≈ 70ms). The scene renders at about 30 fps and only while "En vivo" is visible.
- **Camera presets** (Dojo, Cenital, Seguir): tween over 420ms with an ease-out-quart curve. Orbit damping is 0.12. "Seguir" chases the robot with per-frame lerp.
- **UI transitions:** 0.12–0.2s on the standard ease, `cubic-bezier(.16,1,.3,1)`.
- **Reduced motion:** every CSS transition and animation is removed, the scene damping goes to 1 (values snap), and camera presets jump instantly.

### Missing Data
Every value without fresh data renders "—", including those in the canvas stats and the event-log placeholder. When robot telemetry is stale, the readings are replaced with missing values, never zeros, and the column dims to faint. A pending stage shows "Esperando al robot" centered over the ring.

## Do's and Don'ts

### Do:
- **Do** give every module exactly one shared 1px rule (rule) with its neighbors, and keep secondary views in the same mosaic.
- **Do** reserve wheel yellow (car) for things the operator commanded: heading target, trail, glove or manual order, pending gains, sent commands.
- **Do** draw signals as tawara hairlines and give color only to Kalman (fusa) and the ESP32 filter (umber-hi).
- **Do** set every live number in Barlow Semi Condensed with tabular figures, a decimal comma and a typographic minus.
- **Do** render missing or stale values as "—" and dim stale readings to faint.
- **Do** keep rectangles at 2px radius or less, and use full circles where the world is round (ring, pad, dots).
- **Do** step edge warnings at 12 cm (amber) and 6 cm (red), the same on the arc, the readout and the log.
- **Do** honor prefers-reduced-motion by snapping values and cameras.

### Don't:
- **Don't** tile the console into rounded dark cards with the ring as one tile among them.
- **Don't** use yellow for emphasis, links, warnings or branding.
- **Don't** use stop red (stop) for data, or signal red (red) for the stop control.
- **Don't** add shadows to modules, panels, pills or buttons. Elevation is only for the drive knob and the toast.
- **Don't** show 0 for a value that has not arrived.
- **Don't** add entrance animations or motion that is not damped.
