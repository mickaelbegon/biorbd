# Standard for the biorbd videos

Every video of the series follows this document. `STANDARD.fr.md` is the French version; keep both in sync.
The render layer (`common/layer.py`) already handles the logo, slow-down, text transitions, translation,
the final card and the frame audit, so a scene only contains the content.

## 1. Data rules (D)

- **D1 — Real computations only.** Every number, curve, pose or trajectory comes from a real biorbd call
  made by a `generate_<topic>_data.py` script and stored in `data/<topic>_*.npz`. Never type a result by hand.
- **D2 — Displayed numbers come from the `.npz`.** Format them in the scene from the loaded arrays with one
  template per quantity (for example `f"{value:.2f}"`), never as literals.
- **D3 — Label what is not biorbd output.** Synthetic data (simulated motion, added noise), redrawn interfaces
  and hand-written algorithms (an RK4 integrator, Gauss-Newton) carry an on-screen label
  (`bio_style.synthetic_tag`).
- **D4 — Verified on this version.** Displayed code and API names are checked against biorbd 1.12.3
  (`grep` in `include/` and `binding/python3/`). The generator scripts must run in the `biorbd_anim` environment.
- **D5 — Displayed code is the executed code.** The lines of the code panel are the ones the generator runs
  (copy them, or read them from the generator's source). No shortened pseudo-code.
- **D6 — If it does not work, stop.** Do not invent a result: write what was tried in `notes/<topic>.md`.

## 2. Anatomy of a scene (A)

- **A1 — Title and subtitle** at the top left (`bio_style.title_block`): title 44 pt, subtitle 28 pt.
- **A2 — Visuals on the left** (x from -6.8 to -0.5). **A3 — Code panel on the right**
  (`bio_style.code_panel`), titled "biorbd code", with the caption **above** the code box.
- **A4 — One closing sentence** at the bottom (`bio_style.footer`), a whole sentence.
- **A5 — Content lasts 10-20 s at native speed and makes at most 2 points.** The layer slows everything
  down by x1.6 and holds the last frame for 2.5 s, then shows the 4.5 s "Go further" card. Never add
  your own final pause or end card.
- **A6 — When a quantity changes, draw a ghost curve and a difference axis:** keep the previous curve
  faded (opacity 0.3) and show the difference on a small axis below in `C_DIFF`.
- **A7 — Colour roles are fixed** (see `bio_style.py`): `C_TITLE` the object explained, `C_MODEL` what biorbd
  computes, `C_DATA` input or reference data, `C_DIFF` differences, `C_SYNTH` synthetic labels, `C_OK` passed checks.
  Fonts: `Segoe UI` for text, `Consolas` for code (set by the layer when you do not choose).

## 3. Text rules (T)

- **T1 — Whole sentences** ("Sentences", not fragments), one idea each. **T2 — No `Paragraph`, no LaTeX / `Tex` /
  `MathTex`.** Write formulas as plain `Text`.
- **T3 — Numbers use one template** per scene so the FR decimal comma is applied uniformly (the layer does it).
- **T4 — Avoid `t2c`, slices and colouring inside a translatable string**: the translation changes the
  character positions. Use several `Text` objects instead.
- **T5 — Keep room for French**: it is about 20 % longer. Do not size boxes on the English text.
- **T6 — Text in code font is never translated.** Use `font=bio_style.CODE_FONT` for code, names and paths.
- **T7 — Use `Write`, `FadeIn`, `ReplacementTransform` freely**: the layer turns Write/letter-by-letter into a
  fade and a text-to-text morph into fade-out then fade-in, without overlap.
- **T8 — Do not touch the logo.** Do not use `self.clear()` or fade the logo; fade objects individually
  or in a `Group` that excludes it.

## 4. Files

| File | Owner |
|---|---|
| `anim_<topic>.py` | one scene class per file, class name `Anim<Topic>` (CamelCase) |
| `generate_<topic>_data.py` | writes `data/<topic>_*.npz` |
| `data/<topic>_*.npz` | committed (small) |
| `notes/<topic>.md` | what was checked, what failed, open questions |
| `i18n/fr_<topic>.json` | French table for this scene |
| `catalog_part_<topic>.json` | this scene's catalog entry (merged by `tools/merge_catalog.py`) |

`anim__*.py` are layer self-tests, not part of the series. Never write files at the repository root.

## 5. Catalog entry

One entry per scene in `catalog.json`:

```json
{
  "id": "03", "slug": "forward-kinematics", "scene": "AnimForwardKinematics", "level": 1,
  "section": {"en": "Kinematics", "fr": "Cinématique"},
  "title": {"en": "...", "fr": "..."}, "description": {"en": "...", "fr": "..."},
  "notes": {"en": "...", "fr": "..."},
  "links": [{"label": {"en": "Example", "fr": "Exemple"}, "path": "examples/python3/forward_kinematics.py", "lines": [24, 45]}]
}
```

2 to 5 links to examples and code; every `path` exists and `lines` are inside the file (checked by
`tools/validate_catalog.py`; find them with `grep -n`).

## 6. Translation flow

1. Render in English: `python tools/render_series.py <Scene> --lang en`. The report
   `media/<Scene>_en/report_<Scene>_en.json` lists every key (`keys`).
2. Write `i18n/fr_<topic>.json`: key = the English string with numbers replaced by `{0}`, `{1}`;
   value = the French string with the same placeholders. Use the glossary in `i18n/GLOSSARY.md`.
3. Render with `--lang fr --strict`: missing keys fail the run.

## 7. Commands

```bash
python tools/fetch_logo.py                       # once (the PNG is not committed)
python tools/render_series.py --list
python tools/render_series.py <Scene> --lang both --quality 480p15   # quick check
python tools/render_series.py --all --lang both --jobs 3 --strict --collect
python tools/extract_frames.py media/<Scene>_fr/videos/anim_<topic>/1080p30/<Scene>.mp4
python tools/validate_catalog.py
black -t py311 -l120 docs/animations
```

Environments: rendering uses Python 3.11 with `manim` 0.21 (`envs/manim311`); data generation uses the
conda environment `biorbd_anim` (Python 3.12, biorbd 1.12.3).

## 8. Quality checklist and definition of done

- [ ] Data come from `.npz` made by a real biorbd run (D1-D2); synthetic parts are labelled (D3).
- [ ] Displayed code equals executed code and exists in this version (D4-D5).
- [ ] Anatomy respected (A1-A7), 10-20 s of content, 2 points at most.
- [ ] Text rules respected (T1-T8); French table complete.
- [ ] `render_series.py --strict` passes for EN and FR at 1080p30 (no audit finding).
- [ ] Frames checked with `extract_frames.py` (about 1 s, 35 %, 65 %, before the card, middle of the card):
      PNG files were actually looked at; nothing outside the frame, overlapping or touching the logo, even with French text.
- [ ] Catalog entry validated. `black -t py311 -l120` run after the last edit.

A scene is **done** only when every box is ticked and `notes/<topic>.md` states what remains unverified.
