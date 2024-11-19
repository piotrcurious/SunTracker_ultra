import os
import numpy as np
from sympy import symbols, GF, isprime, Poly
from sympy.abc import x
from typing import List, Tuple

# Ensure the output directory exists
OUTPUT_DIR = "results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

def generate_discrete_sine_field(p: int, n_points: int) -> List[int]:
    """
    Generate discrete sine values modulo p.
    
    :param p: Prime modulus for the finite field.
    :param n_points: Number of points in the sine approximation.
    :return: List of sine values mod p.
    """
    if not isprime(p):
        raise ValueError(f"Field modulus {p} must be prime.")
    
    sine_values = [
        int(round(p * (0.5 * (1 + np.sin((2 * np.pi * i) / n_points))))) % p
        for i in range(n_points)
    ]
    return sine_values

def generate_lfsr_polynomials(length: int, field_modulus: int) -> List[List[int]]:
    """
    Generate all primitive polynomials of a given length in a finite field.
    
    :param length: Degree of the polynomial.
    :param field_modulus: Prime modulus for the finite field.
    :return: List of primitive polynomials (as lists of coefficients).
    """
    polys = []
    for i in range(1 << (length + 1)):
        coeffs = [(i >> j) & 1 for j in range(length + 1)]
        if coeffs[-1] == 0:  # Highest-degree coefficient must be non-zero
            continue
        
        poly = Poly(sum(c * x**j for j, c in enumerate(coeffs)), modulus=field_modulus)
        if is_primitive_polynomial(poly, field_modulus):
            polys.append(coeffs)
    return polys

def is_primitive_polynomial(poly: Poly, field_modulus: int) -> bool:
    """
    Check if a polynomial is primitive in the given finite field.
    
    :param poly: Polynomial to check.
    :param field_modulus: Prime modulus for the finite field.
    :return: True if the polynomial is primitive, False otherwise.
    """
    degree = poly.degree()
    order = (field_modulus**degree) - 1  # Calculate the order of the field
    for d in range(1, order):
        if order % d == 0 and pow(x, d, poly.as_expr()) % field_modulus == 1:
            return False
    return True

def generate_lfsr_sequence(coeffs: List[int], length: int, modulus: int) -> List[int]:
    """
    Generate an LFSR sequence based on coefficients.
    
    :param coeffs: Polynomial coefficients for the LFSR.
    :param length: Length of the sequence to generate.
    :param modulus: Modulus for the finite field.
    :return: List of LFSR output values.
    """
    state = [1] * len(coeffs)  # Initial state
    output = []
    for _ in range(length):
        output.append(state[-1])  # Output is the last bit
        feedback = sum(c * state[i] for i, c in enumerate(coeffs[:-1])) % modulus
        state = [feedback] + state[:-1]
    return output

def generate_arduino_code(lfsr1: List[int], lfsr2: List[int], modulus: int) -> str:
    """
    Generate Arduino-compatible C++ code for two LFSRs.
    
    :param lfsr1: Polynomial for LFSR1.
    :param lfsr2: Polynomial for LFSR2.
    :param modulus: Modulus used in the field.
    :return: String containing the Arduino header file.
    """
    return f"""\
#ifndef LFSR_PAIR_H
#define LFSR_PAIR_H

// Modulus for the field
#define MODULUS {modulus}

// LFSR1 coefficients
unsigned int lfsr1[] = {{{', '.join(map(str, lfsr1))}}};

// LFSR2 coefficients
unsigned int lfsr2[] = {{{', '.join(map(str, lfsr2))}}};

#endif
"""

def save_arduino_code(lfsr1: List[int], lfsr2: List[int], modulus: int) -> None:
    """
    Save Arduino code to a file.
    
    :param lfsr1: Polynomial for LFSR1.
    :param lfsr2: Polynomial for LFSR2.
    :param modulus: Modulus used in the field.
    """
    filename = os.path.join(OUTPUT_DIR, f"LFSR_{lfsr1}_{lfsr2}.h")
    with open(filename, "w") as f:
        f.write(generate_arduino_code(lfsr1, lfsr2, modulus))
    print(f"Saved Arduino code to {filename}")

def search_lfsr_pairs(field_modulus: int, lfsr_length: int, n_points: int) -> None:
    """
    Search for LFSR pairs that generate the sine function.
    
    :param field_modulus: Modulus for the finite field.
    :param lfsr_length: Length of LFSRs to search.
    :param n_points: Number of sine approximation points.
    """
    print("Generating discrete sine values...")
    sine_values = generate_discrete_sine_field(field_modulus, n_points)
    print(f"Generated sine values: {sine_values}")
    
    print("Generating LFSR polynomials...")
    lfsr_polys = generate_lfsr_polynomials(lfsr_length, field_modulus)
    print(f"Generated {len(lfsr_polys)} LFSR polynomials.")
    
    print("Searching for matching LFSR pairs...")
    for poly1 in lfsr_polys:
        for poly2 in lfsr_polys:
            lfsr1_seq = generate_lfsr_sequence(poly1, n_points, field_modulus)
            lfsr2_seq = generate_lfsr_sequence(poly2, n_points, field_modulus)
            combined_seq = [(lfsr1_seq[i] + lfsr2_seq[i]) % field_modulus for i in range(n_points)]
            
            if combined_seq == sine_values:
                print(f"Found matching pair: {poly1}, {poly2}")
                save_arduino_code(poly1, poly2, field_modulus)
                return  # Stop searching after finding the first match

# Parameters
FIELD_MODULUS = 31  # Prime modulus
LFSR_LENGTH = 5     # Degree of polynomials
N_POINTS = 32       # Number of sine points

# Run the search
search_lfsr_pairs(FIELD_MODULUS, LFSR_LENGTH, N_POINTS)
