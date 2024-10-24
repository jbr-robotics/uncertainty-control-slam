#!/bin/bash

FILENAME=$(find /root/.cargo/registry/src -type f -name "biguint.rs" | grep "num-bigint-0.3.0/src/biguint.rs")

if [[ -z "$FILENAME" ]]; then
    echo "Error: biguint.rs file not found!"
    exit 1
fi

declare -A changes=(
    [198]=".div_ceil(big_digit::BITS.into())"
    [1700]="let root_scale = extra_bits.div_ceil(n64);"
    [2119]=".div_ceil(u64::from(bits))"
    [2147]=".div_ceil(u64::from(bits))"
)

for line_number in "${!changes[@]}"; do
    new_content="${changes[$line_number]}"
    echo "Modifying line $line_number in $FILENAME to: $new_content"
    sed -i "${line_number}c\\${new_content}" "$FILENAME"
done

echo "All changes applied successfully."
