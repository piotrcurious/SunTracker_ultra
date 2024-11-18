import math
import itertools
import os
import json
import numpy as np
from collections import defaultdict

SEARCH_DIR = "lfsr_search_results"
os.makedirs(SEARCH_DIR, exist_ok=True)

# Known primitive polynomials for common bit widths
PRIMITIVE_POLYNOMIALS = {
    16: [
        [16, 15, 13, 4],  # x^16 + x^15 + x^13 + x^4 + 1
        [16, 14, 13, 11], # x^16 + x^14 + x^13 + x^11 + 1
        [16, 15, 14, 1],  # x^16 + x^15 + x^14 + x + 1
        [16, 15, 13, 1],  # x^16 + x^15 + x^13 + x + 1
        [16, 15, 12, 1],  # x^16 + x^15 + x^12 + x + 1
        [16, 14, 9, 1],   # x^16 + x^14 + x^9 + x + 1
        [16, 13, 12, 1],  # x^16 + x^13 + x^12 + x + 1
        [16, 15, 11, 2],  # x^16 + x^15 + x^11 + x^2 + 1
    ]
}

def get_primitive_polynomials(bit_width):
    """Get list of known primitive polynomials for given bit width."""
    if bit_width not in PRIMITIVE_POLYNOMIALS:
        raise ValueError(f"No known primitive polynomials for bit width {bit_width}")
    return PRIMITIVE_POLYNOMIALS[bit_width]

def generate_tap_permutations(polynomial, min_taps=2):
    """Generate all valid permutations of polynomial taps."""
    base_taps = [t for t in polynomial if t != polynomial[0]]  # Remove highest degree term
    permutations = []
    
    # Generate all possible combinations of taps
    for r in range(min_taps, len(base_taps) + 1):
        for taps in itertools.combinations(base_taps, r):
            permutations.append(list(taps))
    
    return permutations

def validate_taps(bit_width, taps):
    """Validate LFSR taps."""
    if not taps:
        raise ValueError("Taps list cannot be empty")
    if not all(1 <= tap <= bit_width for tap in taps):
        raise ValueError(f"Invalid taps: {taps}. Taps must be in range 1 to {bit_width}.")

def generate_lfsr_sequence(bit_width, taps, seed, length, phase_shift=0):
    """Generate LFSR sequence with phase shift support."""
    validate_taps(bit_width, taps)
    if seed == 0:
        raise ValueError("Seed cannot be 0 for LFSR")
    
    max_value = (1 << bit_width) - 1
    sequence = []
    lfsr = seed & max_value  # Ensure seed is within valid range

    # Apply initial phase shift
    for _ in range(phase_shift):
        new_bit = 0
        for tap in taps:
            new_bit ^= (lfsr >> (bit_width - tap)) & 1
        lfsr = ((lfsr << 1) | new_bit) & max_value

    # Generate sequence
    seen_states = set()
    for _ in range(length):
        if lfsr in seen_states and len(sequence) < max_value:
            raise ValueError(f"LFSR sequence repeats after {len(sequence)} states")
        
        sequence.append(lfsr)
        seen_states.add(lfsr)
        
        new_bit = 0
        for tap in taps:
            new_bit ^= (lfsr >> (bit_width - tap)) & 1
        lfsr = ((lfsr << 1) | new_bit) & max_value

    return sequence

def normalize_sequence(sequence, bit_width):
    """Normalize LFSR sequence to [0, 1] range."""
    max_state = (1 << bit_width) - 1
    return [x / max_state for x in sequence]

def calculate_accuracy(sine_seq, cosine_seq):
    """Calculate accuracy metrics for sine/cosine pair."""
    phases = np.array([x * 2 * np.pi for x in sine_seq])
    
    # Calculate expected values
    expected_sine = np.sin(phases)
    expected_cosine = np.cos(phases)
    
    # Scale actual values from [0,1] to [-1,1]
    actual_sine = 2 * np.array(sine_seq) - 1
    actual_cosine = 2 * np.array(cosine_seq) - 1
    
    # Calculate errors
    sine_rmse = np.sqrt(np.mean((expected_sine - actual_sine) ** 2))
    cosine_rmse = np.sqrt(np.mean((expected_cosine - actual_cosine) ** 2))
    
    # Calculate phase error using arctangent
    actual_phases = np.arctan2(actual_sine, actual_cosine)
    phase_error = np.mean(np.abs(np.mod(actual_phases - phases + np.pi, 2 * np.pi) - np.pi))
    
    max_error = float(max(np.max(np.abs(expected_sine - actual_sine)),
                         np.max(np.abs(expected_cosine - actual_cosine))))
    
    return {
        "sine_rmse": float(sine_rmse),
        "cosine_rmse": float(cosine_rmse),
        "phase_error": float(phase_error),
        "max_error": max_error,
        "quadrature_error": float(np.mean(np.abs(actual_sine**2 + actual_cosine**2 - 1)))
    }

def verify_primitive_polynomial(bit_width, taps, test_length=None):
    """Verify if a polynomial is primitive by checking period length."""
    if test_length is None:
        test_length = (1 << bit_width) - 1
    
    try:
        sequence = generate_lfsr_sequence(bit_width, taps, 1, test_length)
        unique_states = set(sequence)
        # Check if sequence has maximum period length
        if len(unique_states) == test_length:
            # Verify sequence properties
            sequence_array = np.array(sequence)
            autocorr = np.correlate(sequence_array, sequence_array, mode='full')
            normalized_autocorr = autocorr[len(sequence_array)-1:] / autocorr[len(sequence_array)-1]
            # Check for good statistical properties
            if np.all(np.abs(normalized_autocorr[1:]) < 0.3):  # Low autocorrelation
                return True
    except ValueError:
        pass
    return False

def optimize_phase_shift(sine_taps, cosine_taps, bit_width, length):
    """Find optimal phase shift between sine and cosine LFSRs."""
    period = (1 << bit_width) - 1
    best_shift = 0
    best_error = float('inf')
    
    # Test multiple phase shifts for better accuracy
    num_test_points = 16  # Increased from 8
    test_shifts = [int(i * period / num_test_points) for i in range(num_test_points)]
    
    for shift in test_shifts:
        try:
            sine_seq = normalize_sequence(
                generate_lfsr_sequence(bit_width, sine_taps, 1, length), bit_width)
            cosine_seq = normalize_sequence(
                generate_lfsr_sequence(bit_width, cosine_taps, 1, length, shift), bit_width)
            
            accuracy = calculate_accuracy(sine_seq, cosine_seq)
            # Consider both amplitude and phase errors
            total_error = (accuracy["sine_rmse"] + accuracy["cosine_rmse"] + 
                         accuracy["phase_error"] + accuracy["quadrature_error"])
            
            if total_error < best_error:
                best_error = total_error
                best_shift = shift
        except ValueError:
            continue
            
    return best_shift

def search_lfsr_pairs(bit_width, length, accuracy_threshold=0.01, min_taps=2):
    """Search for LFSR pairs using permutations of primitive polynomials."""
    results = []
    primitive_polys = get_primitive_polynomials(bit_width)
    
    # Generate all valid tap permutations
    all_tap_combinations = []
    for poly in primitive_polys:
        permutations = generate_tap_permutations(poly, min_taps)
        all_tap_combinations.extend(permutations)
    
    # Remove duplicates
    all_tap_combinations = [list(x) for x in set(tuple(sorted(taps)) for taps in all_tap_combinations)]
    
    print(f"Testing {len(all_tap_combinations)} tap combinations...")
    
    # Verify primitiveness of tap combinations
    verified_taps = []
    for taps in all_tap_combinations:
        if verify_primitive_polynomial(bit_width, taps):
            verified_taps.append(taps)
    
    print(f"Found {len(verified_taps)} verified primitive tap combinations")
    
    # Search for pairs
    total_combinations = len(verified_taps) * (len(verified_taps) - 1) // 2
    combination_count = 0
    
    for i, sine_taps in enumerate(verified_taps):
        for cosine_taps in verified_taps[i+1:]:
            combination_count += 1
            if combination_count % 100 == 0:
                print(f"Progress: {combination_count}/{total_combinations} combinations tested")
            
            try:
                phase_shift = optimize_phase_shift(sine_taps, cosine_taps, bit_width, length)
                
                sine_seq = normalize_sequence(
                    generate_lfsr_sequence(bit_width, sine_taps, 1, length), bit_width)
                cosine_seq = normalize_sequence(
                    generate_lfsr_sequence(bit_width, cosine_taps, 1, length, phase_shift), 
                    bit_width)
                
                accuracy = calculate_accuracy(sine_seq, cosine_seq)
                
                if accuracy["max_error"] <= accuracy_threshold:
                    results.append({
                        "sine_taps": sine_taps,
                        "cosine_taps": cosine_taps,
                        "phase_shift": phase_shift,
                        "accuracy": accuracy
                    })
                    print(f"\nFound valid pair:")
                    print(f"Sine taps: {sine_taps}")
                    print(f"Cosine taps: {cosine_taps}")
                    print(f"Max error: {accuracy['max_error']:.6f}")
            except ValueError as e:
                continue
                
    results.sort(key=lambda x: x["accuracy"]["max_error"])
    return results

def generate_arduino_code(result):
    """Generate optimized Arduino code for LFSR implementation."""
    sine_mask = sum(1 << (16 - tap) for tap in result["sine_taps"])
    cosine_mask = sum(1 << (16 - tap) for tap in result["cosine_taps"])
    
    code = f"""
// LFSR-based Sine/Cosine Generator
// Maximum error: {result["accuracy"]["max_error"]:.6f}
// Phase error: {result["accuracy"]["phase_error"]:.6f} radians

class LFSRTrigGenerator {{
private:
    static const uint16_t SINE_MASK = 0x{sine_mask:04X};   // Taps: {result["sine_taps"]}
    static const uint16_t COSINE_MASK = 0x{cosine_mask:04X}; // Taps: {result["cosine_taps"]}
    static const uint16_t PHASE_SHIFT = {result["phase_shift"]};
    
    uint16_t sine_state;
    uint16_t cosine_state;
    
    static inline uint16_t nextState(uint16_t state, uint16_t mask) {{
        uint16_t newBit = __builtin_parity(state & mask);
        return (state << 1) | newBit;
    }}
    
public:
    LFSRTrigGenerator(uint16_t initial_state = 1) : 
        sine_state(initial_state),
        cosine_state(initial_state) {{
        // Apply phase shift to cosine
        for(uint16_t i = 0; i < PHASE_SHIFT; i++) {{
            cosine_state = nextState(cosine_state, COSINE_MASK);
        }}
    }}
    
    // Get next sine/cosine pair (normalized to [-1, 1])
    void next(float& sine, float& cosine) {{
        sine_state = nextState(sine_state, SINE_MASK);
        cosine_state = nextState(cosine_state, COSINE_MASK);
        
        // Convert to [-1, 1] range
        sine = (float(sine_state) / 32768.0f) * 2.0f - 1.0f;
        cosine = (float(cosine_state) / 32768.0f) * 2.0f - 1.0f;
    }}
    
    // Get angle in degrees [0, 360)
    float getAngle() {{
        return (float(sine_state) / 65535.0f) * 360.0f;
    }}
    
    // Reset generator to initial state
    void reset(uint16_t initial_state = 1) {{
        sine_state = initial_state;
        cosine_state = initial_state;
        for(uint16_t i = 0; i < PHASE_SHIFT; i++) {{
            cosine_state = nextState(cosine_state, COSINE_MASK);
        }}
    }}
}};
"""
    return code

def save_search_results(results, filename):
    """Save search results to a JSON file."""
    with open(filename, "w") as f:
        json.dump(results, f, indent=4)

def load_search_results(filename):
    """Load search results from a JSON file."""
    if os.path.exists(filename):
        with open(filename, "r") as f:
            return json.load(f)
    return []

if __name__ == "__main__":
    BIT_WIDTH = 16
    SEQUENCE_LENGTH = 1024
    ACCURACY_THRESHOLD = 0.01
    MIN_TAPS = 2
    OUTPUT_FILE = os.path.join(SEARCH_DIR, f"lfsr_pairs_{BIT_WIDTH}bit.json")

    print("\nLFSR Sine/Cosine Generator Search")
    print("================================")
    print(f"Bit width: {BIT_WIDTH}")
    print(f"Sequence length: {SEQUENCE_LENGTH}")
    print(f"Accuracy threshold: {ACCURACY_THRESHOLD}")
    print(f"Minimum taps: {MIN_TAPS}")
    print("================================\n")

    print("Searching for optimal LFSR pairs...")</antArtifact>
  try:
        results = search_lfsr_pairs(BIT_WIDTH, SEQUENCE_LENGTH, ACCURACY_THRESHOLD, MIN_TAPS)
        
        if results:
            print("\nSearch Results Summary:")
            print("=======================")
            print(f"Found {len(results)} valid LFSR pairs")
            
            best_result = results[0]
            print("\nBest Configuration:")
            print(f"Sine taps: {best_result['sine_taps']}")
            print(f"Cosine taps: {best_result['cosine_taps']}")
            print(f"Phase shift: {best_result['phase_shift']}")
            print("\nAccuracy Metrics:")
            print(f"Maximum error: {best_result['accuracy']['max_error']:.6f}")
            print(f"Sine RMSE: {best_result['accuracy']['sine_rmse']:.6f}")
            print(f"Cosine RMSE: {best_result['accuracy']['cosine_rmse']:.6f}")
            print(f"Phase error: {best_result['accuracy']['phase_error']:.6f} radians")
            print(f"Quadrature error: {best_result['accuracy']['quadrature_error']:.6f}")
            
            # Generate visualization
            generate_visualization(best_result, BIT_WIDTH, SEQUENCE_LENGTH)
            
            # Generate and save Arduino code
            arduino_code = generate_arduino_code(best_result)
            arduino_file = os.path.join(SEARCH_DIR, "lfsr_trig.h")
            with open(arduino_file, "w") as f:
                f.write(arduino_code)
            print(f"\nArduino code saved to {arduino_file}")
            
            # Save search results
            save_search_results(results, OUTPUT_FILE)
            print(f"Search results saved to {OUTPUT_FILE}")
            
            # Generate detailed analysis report
            generate_analysis_report(results, os.path.join(SEARCH_DIR, "analysis_report.txt"))
        else:
            print("\nNo suitable LFSR pairs found meeting the accuracy threshold.")
            print("Try adjusting the following parameters:")
            print("1. Increase accuracy threshold")
            print("2. Decrease sequence length")
            print("3. Modify minimum taps requirement")
            
    except Exception as e:
        print(f"\nError during search: {str(e)}")
        raise

def generate_visualization(result, bit_width, length):
    """Generate visualization of LFSR output."""
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle
        
        # Generate sequences
        sine_seq = normalize_sequence(
            generate_lfsr_sequence(bit_width, result['sine_taps'], 1, length), bit_width)
        cosine_seq = normalize_sequence(
            generate_lfsr_sequence(bit_width, result['cosine_taps'], 1, length, 
                                 result['phase_shift']), bit_width)
        
        # Create figure with multiple subplots
        fig = plt.figure(figsize=(15, 10))
        
        # Time domain plot
        ax1 = plt.subplot(221)
        x = np.arange(length) / length * 2 * np.pi
        ax1.plot(x, 2*np.array(sine_seq)-1, 'b-', label='LFSR Sine', alpha=0.7)
        ax1.plot(x, 2*np.array(cosine_seq)-1, 'r-', label='LFSR Cosine', alpha=0.7)
        ax1.plot(x, np.sin(x), 'b--', label='Ideal Sine', alpha=0.3)
        ax1.plot(x, np.cos(x), 'r--', label='Ideal Cosine', alpha=0.3)
        ax1.set_title('Time Domain Comparison')
        ax1.set_xlabel('Phase (radians)')
        ax1.set_ylabel('Amplitude')
        ax1.grid(True)
        ax1.legend()
        
        # Phase plot
        ax2 = plt.subplot(222)
        actual_sine = 2*np.array(sine_seq)-1
        actual_cosine = 2*np.array(cosine_seq)-1
        ax2.plot(actual_cosine, actual_sine, 'b.', alpha=0.3, label='LFSR')
        circle = Circle((0, 0), 1, fill=False, color='r', linestyle='--', label='Unit Circle')
        ax2.add_patch(circle)
        ax2.set_aspect('equal')
        ax2.set_title('Phase Space Plot')
        ax2.set_xlabel('Cosine')
        ax2.set_ylabel('Sine')
        ax2.grid(True)
        ax2.legend()
        
        # Error plot
        ax3 = plt.subplot(223)
        phase = x
        sine_error = actual_sine - np.sin(phase)
        cosine_error = actual_cosine - np.cos(phase)
        ax3.plot(phase, sine_error, 'b.', label='Sine Error', alpha=0.5)
        ax3.plot(phase, cosine_error, 'r.', label='Cosine Error', alpha=0.5)
        ax3.set_title('Error Distribution')
        ax3.set_xlabel('Phase (radians)')
        ax3.set_ylabel('Error')
        ax3.grid(True)
        ax3.legend()
        
        # Histogram of errors
        ax4 = plt.subplot(224)
        ax4.hist(sine_error, bins=50, alpha=0.5, label='Sine Error', color='blue')
        ax4.hist(cosine_error, bins=50, alpha=0.5, label='Cosine Error', color='red')
        ax4.set_title('Error Histogram')
        ax4.set_xlabel('Error')
        ax4.set_ylabel('Count')
        ax4.grid(True)
        ax4.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(SEARCH_DIR, 'lfsr_analysis.png'))
        plt.close()
        
        print("\nVisualization saved as 'lfsr_analysis.png'")
        
    except ImportError:
        print("Matplotlib not available - skipping visualization")

def generate_analysis_report(results, filename):
    """Generate detailed analysis report of search results."""
    with open(filename, 'w') as f:
        f.write("LFSR Sine/Cosine Generator Analysis Report\n")
        f.write("=========================================\n\n")
        
        f.write(f"Total valid pairs found: {len(results)}\n\n")
        
        for i, result in enumerate(results):
            f.write(f"Configuration #{i+1}:\n")
            f.write("-----------------\n")
            f.write(f"Sine taps: {result['sine_taps']}\n")
            f.write(f"Cosine taps: {result['cosine_taps']}\n")
            f.write(f"Phase shift: {result['phase_shift']}\n")
            f.write("\nAccuracy Metrics:\n")
            f.write(f"Maximum error: {result['accuracy']['max_error']:.6f}\n")
            f.write(f"Sine RMSE: {result['accuracy']['sine_rmse']:.6f}\n")
            f.write(f"Cosine RMSE: {result['accuracy']['cosine_rmse']:.6f}\n")
            f.write(f"Phase error: {result['accuracy']['phase_error']:.6f} radians\n")
            f.write(f"Quadrature error: {result['accuracy']['quadrature_error']:.6f}\n\n")
            
            # Calculate tap spacing statistics
            sine_spacings = np.diff(sorted(result['sine_taps']))
            cosine_spacings = np.diff(sorted(result['cosine_taps']))
            
            f.write("Tap Analysis:\n")
            f.write(f"Sine tap count: {len(result['sine_taps'])}\n")
            f.write(f"Sine tap spacing: min={np.min(sine_spacings)}, max={np.max(sine_spacings)}, "
                   f"mean={np.mean(sine_spacings):.2f}\n")
            f.write(f"Cosine tap count: {len(result['cosine_taps'])}\n")
            f.write(f"Cosine tap spacing: min={np.min(cosine_spacings)}, max={np.max(cosine_spacings)}, "
                   f"mean={np.mean(cosine_spacings):.2f}\n\n")
            
        f.write("\nAnalysis Summary:\n")
        f.write("----------------\n")
        all_errors = [r['accuracy']['max_error'] for r in results]
        f.write(f"Best error: {min(all_errors):.6f}\n")
        f.write(f"Worst error: {max(all_errors):.6f}\n")
        f.write(f"Mean error: {np.mean(all_errors):.6f}\n")
        f.write(f"Error std dev: {np.std(all_errors):.6f}\n")
        
    print(f"\nDetailed analysis report saved to {filename}")

if __name__ == "__main__":
    main()
