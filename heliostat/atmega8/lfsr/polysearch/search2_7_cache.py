import os
import json
from sympy import symbols, Poly, GF
from typing import List, Dict, Any

x = symbols('x')


def log(message: str) -> None:
    print(message)


# Cache utilities
CACHE_DIR = "cache"


def ensure_cache_directory() -> None:
    """Ensure the cache directory exists."""
    if not os.path.exists(CACHE_DIR):
        os.makedirs(CACHE_DIR)


def cache_file_path(length: int, field_modulus: int) -> str:
    """Get the file path for a specific cache entry."""
    return os.path.join(CACHE_DIR, f"lfsr_length_{length}_mod_{field_modulus}.json")


def load_cache(length: int, field_modulus: int) -> Dict[str, Any]:
    """Load cache for a specific LFSR configuration."""
    file_path = cache_file_path(length, field_modulus)
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            return json.load(file)
    return {}


def save_cache(length: int, field_modulus: int, data: Dict[str, Any]) -> None:
    """Save cache for a specific LFSR configuration."""
    file_path = cache_file_path(length, field_modulus)
    with open(file_path, "w") as file:
        json.dump(data, file, indent=2)


# LFSR utilities
def is_irreducible(poly: Poly, field_modulus: int) -> bool:
    """Check if a polynomial is irreducible over a finite field."""
    factors = poly.factor_list(domain=GF(field_modulus))[1]
    return len(factors) == 1  # Single irreducible factor


def is_primitive_polynomial(poly: Poly, field_modulus: int) -> bool:
    """Check if a polynomial is primitive over a finite field."""
    degree = poly.degree()
    order = field_modulus**degree - 1
    elements = {pow(x, k, order) for k in range(order)}
    return len(elements) == order


def generate_lfsr_properties(length: int, field_modulus: int) -> Dict[str, List[List[int]]]:
    """
    Generate properties for all LFSRs of a given length and field modulus.
    Cache the properties to avoid redundant computation.
    """
    cache = load_cache(length, field_modulus)
    if cache:
        log("Loaded LFSR properties from cache.")
        return cache

    properties = {"irreducible": [], "primitive": []}
    log("Computing LFSR properties...")

    for i in range(1 << (length + 1)):
        coeffs = [(i >> j) & 1 for j in range(length + 1)]
        if coeffs[-1] == 0:  # Skip non-monic polynomials
            continue

        poly = Poly(sum(c * x**j for j, c in enumerate(coeffs)), modulus=field_modulus)
        if is_irreducible(poly, field_modulus):
            properties["irreducible"].append(coeffs)
            if is_primitive_polynomial(poly, field_modulus):
                properties["primitive"].append(coeffs)

    save_cache(length, field_modulus, properties)
    log("LFSR properties computed and cached.")
    return properties


def search_lfsr_pairs(field_modulus: int, lfsr_length: int, n_points: int) -> None:
    """
    Search for pairs of LFSRs that, when combined, approximate a sine function.

    :param field_modulus: Field modulus (prime number).
    :param lfsr_length: Length of the LFSR (degree of polynomial).
    :param n_points: Number of discrete points in the sine wave.
    """
    ensure_cache_directory()
    sine_values = [(int((15 + 15 * Poly.cos(2 * 3.14159 * i / n_points)).round())) for i in range(n_points)]
    log(f"Sine values generated: {sine_values}")

    lfsr_properties = generate_lfsr_properties(lfsr_length, field_modulus)
    primitive_lfsrs = lfsr_properties["primitive"]

    log(f"Found {len(primitive_lfsrs)} primitive LFSRs.")

    # Example: Combine primitive LFSRs (placeholder logic for sine approximation)
    for lfsr1 in primitive_lfsrs:
        for lfsr2 in primitive_lfsrs:
            # Combine the LFSRs (example logic)
            combined = [(a + b) % field_modulus for a, b in zip(lfsr1, lfsr2)]
            if combined == sine_values:  # Check if approximation matches
                log(f"Found matching LFSR pair: {lfsr1} and {lfsr2}")
                return

    log("No matching LFSR pairs found.")


# Example Usage
FIELD_MODULUS = 31  # Prime number
LFSR_LENGTH = 5  # Degree of LFSR
N_POINTS = 32  # Number of sine wave points

search_lfsr_pairs(FIELD_MODULUS, LFSR_LENGTH, N_POINTS)
