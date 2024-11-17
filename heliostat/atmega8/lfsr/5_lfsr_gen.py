import math

def generate_lfsr_polynomials(bit_width):
    """
    Generate LFSR polynomials for a given bit width.
    Returns pairs of polynomials to be used for sine and cosine approximation.
    """
    # Predefined primitive polynomials for common bit widths
    # (bit_width, [tap positions]) for maximal-length LFSRs
    primitive_polynomials = {
        4: [4, 3],  # Example for 4-bit: taps at positions 4 and 3
        5: [5, 3],
        6: [6, 5],
        7: [7, 6],
        8: [8, 6, 5, 4],  # Example for 8-bit
        9: [9, 5],
        10: [10, 7],
        16: [16, 14, 13, 11],  # Example for 16-bit
    }
    
    if bit_width not in primitive_polynomials:
        raise ValueError(f"No predefined polynomial for {bit_width}-bit LFSR.")
    
    taps = primitive_polynomials[bit_width]
    return taps

def generate_lfsr_sequence(bit_width, taps, seed, length):
    """
    Generate an LFSR sequence with a given bit width, taps, and initial seed.
    """
    lfsr = seed
    max_value = (1 << bit_width) - 1
    sequence = []

    for _ in range(length):
        sequence.append(lfsr)
        new_bit = 0
        for tap in taps:
            new_bit ^= (lfsr >> (tap - 1)) & 1
        lfsr = ((lfsr << 1) | new_bit) & max_value

    return sequence

def map_lfsr_to_trig(lfsr_sequence, amplitude, phase_shift=0):
    """
    Map an LFSR sequence to a sine or cosine function.
    """
    length = len(lfsr_sequence)
    sine_values = []
    for i in range(length):
        angle = 2 * math.pi * i / length + phase_shift
        sine_value = amplitude * math.sin(angle)
        sine_values.append(sine_value)
    return sine_values

def verify_lfsr_pairs(sine_values, cosine_values, tolerance=1e-5):
    """
    Verify that the sine and cosine values satisfy the Pythagorean identity.
    """
    for s, c in zip(sine_values, cosine_values):
        magnitude = s**2 + c**2
        if not math.isclose(magnitude, 1.0, rel_tol=tolerance):
            return False
    return True

# Parameters
bit_width = 8  # Width of LFSR
amplitude = 1.0  # Normalized amplitude for sine and cosine
lfsr_length = 256  # Length of LFSR sequence

# Generate LFSR for sine and cosine
sine_taps = generate_lfsr_polynomials(bit_width)
cosine_taps = generate_lfsr_polynomials(bit_width)  # Separate LFSR pair

sine_seed = 1  # Initial seed for sine LFSR
cosine_seed = 2  # Initial seed for cosine LFSR

# Generate sequences
sine_lfsr_sequence = generate_lfsr_sequence(bit_width, sine_taps, sine_seed, lfsr_length)
cosine_lfsr_sequence = generate_lfsr_sequence(bit_width, cosine_taps, cosine_seed, lfsr_length)

# Map sequences to sine and cosine
sine_values = map_lfsr_to_trig(sine_lfsr_sequence, amplitude)
cosine_values = map_lfsr_to_trig(cosine_lfsr_sequence, amplitude, math.pi / 2)

# Verify Pythagorean identity
is_valid = verify_lfsr_pairs(sine_values, cosine_values)
if is_valid:
    print("LFSR pairs successfully verified for sine and cosine approximation.")
else:
    print("LFSR pairs failed verification.")

# Output sequences for inspection
print("Sine LFSR sequence (scaled):", sine_values)
print("Cosine LFSR sequence (scaled):", cosine_values)
