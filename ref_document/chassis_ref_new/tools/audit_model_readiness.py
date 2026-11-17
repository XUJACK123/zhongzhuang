"""Recompute CAD/catalogue estimates without hardware access or retraining."""
from pathlib import Path
import datetime
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT.parent / 'fifty_mujoco/runs/realdata_20261117'


def read(path):
    return json.loads(path.read_text(encoding='utf-8-sig'))


def reflected(qk, sign, inertia):
    ratio = 113.4 / 135
    beta = sign * qk + math.pi / 2
    alpha = 2 * math.atan2(math.sin(beta), ratio + math.cos(beta))
    k = .5 + ratio * math.cos(alpha / 2) / (2 * math.sqrt(1 - ratio**2 * math.sin(alpha / 2)**2))
    matrix = [[2 * inertia, inertia / k], [inertia / k, inertia / k**2]]
    examples = []
    for h, knee in [(1., 1.), (1., -1.)]:
        motor_energy = .5 * inertia * (h*h + (h + knee/k)**2)
        full_energy = .5 * (matrix[0][0]*h*h + 2*matrix[0][1]*h*knee + matrix[1][1]*knee*knee)
        diagonal = .5 * (matrix[0][0]*h*h + matrix[1][1]*knee*knee)
        assert abs(motor_energy - full_energy) < 1e-12
        examples.append({'joint_velocity_rad_s': [h, knee], 'motor_kinetic_energy_j': motor_energy,
                         'full_matrix_energy_j': full_energy, 'diagonal_energy_j': diagonal})
    assert matrix[0][0]*matrix[1][1] - matrix[0][1]**2 > 0
    return {'knee_derivative': k, 'matrix_kg_m2': matrix, 'kinetic_energy_checks': examples}


def main():
    evidence = read(RUN / 'evidence_and_spring_calculation.json')
    step = Path(read(ROOT / 'real_data_collection/cad_reference_for_field_measurements_20261116.json')['source_step'])
    digest = hashlib.sha256()
    fields = {key: False for key in ('MASS_DENSITY', 'MASS_MEASURE', 'MOMENT_OF_INERTIA', 'CENTRE_OF_MASS')}
    tail = b''
    with step.open('rb') as stream:
        for chunk in iter(lambda: stream.read(4*1024*1024), b''):
            digest.update(chunk)
            for key in fields:
                fields[key] |= key.encode('ascii') in tail + chunk
            tail = chunk[-64:]
    assert digest.hexdigest() == evidence['step']['sha256']
    cfg = read(RUN / 'final_export_full/manifest.json')['configuration']
    pressure = []
    for diameter in (.006, .010):
        area = math.pi * diameter**2 / 4
        pressure.append({'assumed_effective_rod_diameter_m': diameter,
                         'ideal_force_n_at_150_bar': 15e6 * area,
                         'pressure_bar_for_450_n': 450 / area / 1e5})
    pin = evidence['geometry']['pin_m_from_coarse_120deg_included']
    axial = evidence['geometry']['pin_offset_m']
    full = read(RUN / 'final_summary.json')
    stock_ratio = 3591 / 187
    # A first-order motor approximation gives J_eq = tau_m / |domega/dT|.
    # This is not an independently known rotor inertia or installed-reducer value.
    lumped = .049 / (72 * 2*math.pi / 60)
    report = {
        'generated_at': datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(),
        'evidence_kind': 'CAD and catalogue calculations; no new physical measurements',
        'step': {'path': str(step), 'sha256': digest.hexdigest(), 'mass_field_keywords_present': fields},
        'accepted_inputs': {'mass_kg': 21.9, 'rolling_radius_m': .7/(4*math.pi),
                            'left_radius_is_symmetry_assumption': True, 'body_level_is_assumption': True},
        'geometry': {**evidence['geometry'],
                     'spatial_minus_planar_pin_distance_m': pin-math.sqrt(pin*pin-axial*axial),
                     'axial_offset_does_not_explain_14_mm_residual': True,
                     'external_chain_ratio_CAD': 19/16,
                     'installed_external_chain_identity_verified': False},
        'spring': {'installed_model': 'BG450-080', 'CAD_model': '1BKB0.45-75',
                   'catalogue_proxy': 'BKB0.45', 'exact_model_match_verified': False,
                   'actual_charge_pressure_bar': None, 'conditional_area_pressure_calculations': pressure,
                   'reported_10_mm_is_not_verified_effective_rod_diameter': True,
                   'actual_force_stroke_and_hysteresis_identifiable_from_STEP_alone': False,
                   'source_url': 'https://www.bbkspring.com/standard-gas-spring/compact-nitrogen-spring'},
        'coupled_reflected_inertia': {
            'equation': 'M_add(q) = B(q)^T diag(J_h,J_k) B(q)',
            'B_without_optional_external_chain': 's * [[1,0],[1,1/k(q)]]',
            'assumed_equal_motor_output_inertia_kg_m2': cfg['reflected_inertia']['leg_motor_output_kg_m2'],
            'right': reflected(cfg['standing_default_rad'][1], 1, cfg['reflected_inertia']['leg_motor_output_kg_m2']),
            'left': reflected(cfg['standing_default_rad'][4], -1, cfg['reflected_inertia']['leg_motor_output_kg_m2']),
            'scope': 'planar relative-joint submatrix; floating-base cross terms not included',
            'if_CAD_chain_is_installed_matrix_multiplier': (19/16)**2,
            'status': 'full cross terms computed; frozen training plant still uses diagonal-at-default approximation',
            'true_rotor_inertia_identified': False},
        'motor_catalogue': {
            'damiao': {'family': 'DM-J8009P-2EC', 'rated_torque_nm': 20, 'peak_torque_nm': 40,
                       'integrated_ratio': 9, 'output_encoder_must_not_be_divided_by_9_again': True,
                       'installed_variant_and_group_current_scale_fully_verified': False,
                       'source_url': 'https://www.dmbot.cn/index.php?c=show&id=105'},
            'M3508_P19': {'stock_ratio': stock_ratio, 'configured_ratio': 15.88,
                          'conditional_odometry_scale_error_if_stock_P19': stock_ratio/15.88-1,
                          'rotor_vs_output_units_are_distinct': True,
                          'conditional_first_order_lumped_output_inertia_kg_m2': lumped,
                          'first_order_inertia_equation': '49 ms / (72 rpm/Nm converted to rad/s/Nm)',
                          'inertia_is_catalogue_proxy_not_measured_rotor': True,
                          'installed_ratio_not_overwritten': True,
                          'source_url': 'https://rm-static.djicdn.com/robomasters/public/document/RoboMaster%20M3508%20P19%20Brushless%20DC%20Gear%20Motor%20V1.0.pdf'}},
        'model_function_gaps': [
            'Independent get-up qualification failed: near 7/8 in near starts, full 2/8 in full starts; sensitivity 0/18.',
            'New 350 mm models are stationary rise/balance candidates, not qualified driving or reverse/turn controllers.',
            'Physical support removal, contact and assistance-force evidence are not observable from the existing feedback alone.',
            'Full coupled rotor inertia, measured drag, true spring loops and real IMU/CAN timing are absent from the frozen plant.',
            'Encoder reference conflict and position_lost cause remain unresolved; no physical calibration changed.'],
        'can_compute_from_available_sources': ['CAD link lengths, axes, spring geometric leverage and consistent inverse kinematics',
                                               'catalogue ratings and conditional pressure-area force estimates',
                                               'full reflected inertia matrix as a function of assumed rotor inertia and linkage pose'],
        'cannot_uniquely_compute_from_available_sources': ['actual installed spring pressure and hysteresis',
                                                          'actual shaft torque, friction/backlash and efficiency',
                                                          'true mass distribution/COM/inertia without material/component data',
                                                          'current encoder zero, transmission variant and transport/filter timing'],
        'frozen_results_preserved': {'all_scopes_passed': full['all_scopes_passed'],
                                     'sensitivity_passed': full['heldout_sensitivity_successes'],
                                     'sensitivity_trials': full['heldout_sensitivity_trials']},
        'hardware_accessed': False, 'new_training_started': False, 'physical_flags_promoted': False}
    target = ROOT / 'docs/validation/model_readiness_20261117.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'report': str(target), 'step_hash_matches': True,
                      'pin_residual_mm': evidence['geometry']['residual_m']*1000,
                      'stock_P19_ratio': stock_ratio,
                      'full_reflected_inertia': report['coupled_reflected_inertia']['right']['matrix_kg_m2']}, ensure_ascii=True))


if __name__ == '__main__':
    main()
