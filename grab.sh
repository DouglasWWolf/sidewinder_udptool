dest=bitstream
base=udptool

mkdir $dest 2>/dev/null


cp sidewinder.runs/impl_1/design_1_wrapper.bit ${dest}/${base}.bit
cp sidewinder.runs/impl_1/design_1_wrapper.ltx ${dest}/${base}.ltx

