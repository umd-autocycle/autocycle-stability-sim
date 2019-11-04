from sys import argv
from simulation import simulate
from bikemodel import MeijaardModel
from graphs import plot_params
from controls import PIDPhi, PDPhi

if __name__ == "__main__":
    model = MeijaardModel()
    control_PD = PDPhi(k_p=5, k_d=8)
    control_PID = PIDPhi(k_p=5, k_i=.0000001, k_d=8)
    results1 = simulate(model, (float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4])), float(argv[5]),
                        float(argv[6]), control_method=None, perturbation=None)
    results2 = simulate(model, (float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4])), float(argv[5]),
                        float(argv[6]), control_method=control_PD, perturbation=None)
    results3 = simulate(model, (float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4])), float(argv[5]),
                        float(argv[6]), control_method=control_PID, perturbation=None)

    plot_params('Uncontrolled Bicycle at %.2f m/s' % float(argv[6]), (results1['t'], 'Time (seconds)'),
                (results1['phi'], 'Phi (radians)'), (results1['delta'], 'Delta (radians)'))
    plot_params('PD Controlled Bicycle at %.2f m/s' % float(argv[6]), (results2['t'], 'Time (seconds)'),
                (results2['phi'], 'Phi (radians)'), (results2['delta'], 'Delta (radians)'))
    plot_params('PID Controlled Bicycle at %.2f m/s' % float(argv[6]), (results3['t'], 'Time (seconds)'),
                (results3['phi'], 'Phi (radians)'), (results3['delta'], 'Delta (radians)'))
