# Post an Agie Classic Cut 2S job

CamCraft writes the nominal contour, resource manifest and piece script directly. The job opens with zero erosion steps. It requests cold die steel (material 0001), Berco Cut 0.25 mm wire and a target finish of Ra 0.30 µm. Check the effective work technology on the installed control before assigning any cutting steps. The control calculates the offsets and trims. The CamCraft wire-center preview uses a 0.005 mm representative finishing gap.

## Post from the Wire EDM workspace

1. Mount **AGIECUT Classic 2S (template)** as the setup machine. In the machine editor, open **EDM** and add a condition with ID **AGIE_QN2**, role **rough**, wire diameter **0.25 mm** and preview gap **0.005 mm**. Apply this change to the mounted machine. Choose **Native wire EDM job files** in the post picker.
2. Define axis-aligned rectangular stock at most 350 × 250 mm in plan. Choose a stock thickness of 15, 20, 40 or 80 mm. The stock and work origin together must fit that span. Verify the installed work datum and the available axis travel on Agie Control. CamCraft refuses profiles with a stated X or Y travel limit because its preview cannot check the full resident path against that limit. Put the first, unrotated work origin on the stock top. Its XY position determines the job coordinates.
3. Use one modeled workpiece and create one **Wire Contour** on the complete punch model. Use a convex punch with straight sides, vertical walls and flat ends at the stock planes. Angled faces are supported. Leave at least 0.4 mm of stock around the nominal polygon. Acute corners can need more stock: the post also checks the intersection of the two offset face lines and the swept wire at that corner. CamCraft checks each corner with the measured rough offset for the selected stock thickness. The stock margin includes the 0.125 mm wire radius. Additional bodies, selected subprofiles, holes, steps and concavities are refused. Select a 0.25 mm wire and condition **AGIE_QN2**. Set the work planes to the stock top and bottom, and specify how the punch is captured. The capture setting confirms support for the punch when it is released. Keep the wire vertical. Taper, four-axis, die cuts, burnout, explicit skims, tabs and fixture models are unsupported.
4. Place the centre of a predrilled 2 mm threading hole in the waste. The entire hole must be inside the stock. Allow a perpendicular approach to the interior of a straight outside face with at least 3 mm of lead and at least 3 mm of straight face on each side of the entry. Generate the operation.
5. Open **Post**, choose **One program** and keep the general post settings at their defaults. Under **Wire EDM technology**, set **Separation cut length (mm)** to **1.0**. Zero leaves the separation unspecified and blocks the post. This length applies to the separation segment that Agie Control generates. Check its position, trims and release sequence in the control’s job preview. Enter the parent folder and a fresh **Job folder name** such as `PUNCH40`. Use up to eight uppercase letters or digits, beginning with a letter. The post publishes the entire directory after validation. To retain an earlier job, give the new one a different name.

### QN2 offsets by stock thickness

Agie Control calculated these wire-centre offsets on the straight outside faces of a synthetic rectangular punch. All dimensions are in millimetres. The native job sends nominal geometry. The resident technology applies each offset. The post review names the offsets of the selected thickness. Check them in the control before GRAFICHECK and FORMALCHECK.

| Stock thickness | Rough | Trim 1 | Trim 2 | Trim 3 |
| ---: | ---: | ---: | ---: | ---: |
| 15 | 0.245 | 0.155 | 0.140 | 0.134 |
| 20 | 0.245 | 0.155 | 0.140 | 0.134 |
| 40 | 0.260 | 0.170 | 0.140 | 0.130 |
| 80 | 0.266 | 0.176 | 0.143 | 0.133 |

The clearance check rounds the rough offset plus the wire radius up to the next tenth of a millimetre. For these four thicknesses this gives 0.4 mm on a straight face. At a pointed corner the post checks the miter for that thickness separately. Other thicknesses are refused. They need their own control measurements.

## Try the synthetic punch

From the project root, run the supplied script with a local CamCraft desktop CLI build.

```sh
cargo run -p camcraft-app --bin camcraft-cli -- script examples/agie/punch.camscript
```

The script imports an original 40 × 20 × 20 mm rectangular punch, places it in 80 × 60 × 20 mm stock and sets the threading point at X−30 Y0. Its post call selects the native separation with `options: {separationLengthMm: 1.0}`. Stock top is Z0 in the job. The `PUNCH40` directory contains the nominal contour in `PUNCH40.ISO`, its import reference in `PUNCH40.SBR` and the piece, start point and requested quality in `PUNCH40.SBL`. Existing destinations are refused. Edit the script’s output name to make another job.

For an oblique entry, run `cargo run -p camcraft-app --bin camcraft-cli -- script examples/agie/oblique-punch.camscript` with a fresh `POLY20` destination. This pentagonal punch has a 3:4 sloping entry face. Its threading hole at X−20.5 Y6 approaches X−12.5 Y0 perpendicular to the middle of that face.

Run `cargo run -p camcraft-app --bin camcraft-cli -- script examples/agie/slope-punch.camscript` for a 7:10 entry face. The `SLOPE20` job keeps the drilled hole at X−20.5 Y6. Its perpendicular entry falls between five-decimal ISO coordinates. CamCraft can move the entry up to 0.00001 mm toward the waste while preserving the lead, separation room and rough-pass clearance. Both jobs open with zero erosion steps, as the rectangular job does.

## Open the job in Agie Control

Keep all three files together. In the control’s file manager, open **Imp/exp** and import the ISO object into **ISO Sys**. Then import the SBL into **SBL Sys**, keeping the matching SBR beside the SBL. In the workpiece list choose **New workpiece → Script Files**, select the imported SBL and confirm with **OK**.

Select the newly created piece. After a repeat import, an earlier piece can retain its machining steps. Match the new work’s ISO geometry and threading hole to the setup. Check that the new work shows **[0]**. At this point **FORMALCHECK** should report undefined technology.

1. Open **Work Details** on the new work. Verify steel **0001**, wire **Berco Cut 0.25**, the stock height you selected (**15**, **20**, **40** or **80 mm**) and the effective work quality. The requested **CCB030** values are **Ra 0.30**, **Te 1.50** and **Tkm 1.00**, with three options disabled. Check the two options the control displays. A resident quality with the same name can hold different values.
2. If any visible value differs, use **Quality → Define** on this work to select the requested values. Confirm the effective settings before proceeding.
3. Open **Technology**, select **TECCUT** and choose **Assign** on the matching row. Inspect the generated steps. The rough and trim offsets must agree with the row of the selected stock thickness in [QN2 offsets by stock thickness](#qn2-offsets-by-stock-thickness), in that order. The post review names the same four offsets. For 20 mm stock they are **0.245, 0.155, 0.140 and 0.134 mm**.
4. Run **FORMALCHECK**. Inspect errors and warnings. Save the piece and run **GRAFICHECK**. Check the installed XY datum and travel, the approach from the hole and compensation outside the punch. Verify the 1 mm separation section, trimming sequence and support for release. CamCraft does not command a manual pause before the section cut.
