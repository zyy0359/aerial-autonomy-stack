# Viewing and Building the ICTAI LaTeX Draft on Windows

This project currently contains the ICTAI draft at:

- `experiments/spray_dqn/ictai2026_initial_draft.tex`
- `experiments/spray_dqn/ictai2026_refs.bib`

## Immediate Preview

If LaTeX is not installed yet, open this generated reading preview first:

- `experiments/spray_dqn/ictai2026_readable_preview.pdf`
- `experiments/spray_dqn/ictai2026_readable_preview_merged.pdf`

This preview is only for reading the content. It is not the official IEEE two-column PDF and should not be submitted.

## Recommended Setup

1. Install a LaTeX distribution:
   - Recommended on Windows: MiKTeX
   - Alternative: TeX Live

2. Install VS Code extension:
   - Open VS Code
   - Install `LaTeX Workshop`

3. Open the repository folder in VS Code:
   - `C:\Users\zyy\Documents\GitHub\aerial-autonomy-stack`

4. Open:
   - `experiments/spray_dqn/ictai2026_initial_draft.tex`

5. Build the PDF:
   - In VS Code, use `Ctrl+Alt+B`
   - Or open Command Palette and run `LaTeX Workshop: Build LaTeX project`

6. View the PDF:
   - LaTeX Workshop usually opens the PDF preview automatically.
   - If not, run `LaTeX Workshop: View LaTeX PDF`.

## Command-Line Build

After installing MiKTeX or TeX Live, run these commands from:

```powershell
cd C:\Users\zyy\Documents\GitHub\aerial-autonomy-stack\experiments\spray_dqn
```

Then build:

```powershell
pdflatex ictai2026_initial_draft.tex
bibtex ictai2026_initial_draft
pdflatex ictai2026_initial_draft.tex
pdflatex ictai2026_initial_draft.tex
```

The output should be:

```text
ictai2026_initial_draft.pdf
```

## Current Local Status

VS Code is installed on this machine, but `pdflatex`, `xelatex`, and `bibtex` were not found in the current PATH during the check. Install MiKTeX or TeX Live first, then restart VS Code/PowerShell so the commands are available.

## ICTAI 2026 Notes

For ICTAI 2026 full-paper submission, keep the draft anonymous:

- Do not include author names, affiliation, acknowledgments, or personal repository paths.
- Use IEEE two-column, 10pt format.
- Keep the full paper within 8 pages.
- Supplementary material can be submitted separately, but the main PDF must stand on its own.
