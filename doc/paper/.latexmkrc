# --- output directory ---
$out_dir = 'build';
$aux_dir = 'build';

# --- emulate aux separation ---
$emulate_aux = 1;

# --- output PDF name ---
$jobname = 'paper';

# --- compiler ---
$pdf_mode = 1;
$pdflatex = 'pdflatex -interaction=nonstopmode -synctex=1 %O %S';

# --- bibliography ---
$bibtex = 'bibtex %O %S';

# --- move PDF to root ---
END {
    if (-e 'build/paper.pdf') {
        rename 'build/paper.pdf', 'paper.pdf';
    }
}

# create build folder
if (!-d 'build') { mkdir 'build'; }
