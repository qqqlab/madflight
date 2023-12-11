# Low-pass Filter

$Output_i = (1-\beta) Output_{i-1} + \beta Sample_i$

$\beta = 1 - e^{-2 \pi \frac{F_0}{F_S}}$

$F_0 = \frac{F_S}{2\pi} e^{\frac{1}{1-\beta}}$


$F_0$ Cutoff frequency (Hz)  
$F_S$ Sample Frequency (Hz)  
$\beta$ Filter factor (0 to 1, higher $\beta$ gives higher $F_0$)  

## Alternate form

$\alpha = 1 - \beta$

$Output_i = \alpha Output_{i-1} + (1-\alpha) Sample_i$

$\alpha = e^{-2 \pi \frac{F_0}{F_S}}$

$F_0 = \frac{F_S}{2\pi} e^{\frac{1}{\alpha}}$

$F_0$ Cutoff frequency (Hz)  
$F_S$ Sample Frequency (Hz)  
$\alpha$ Filter factor (0 to 1, lower $\alpha$ gives higher $F_0$)  
