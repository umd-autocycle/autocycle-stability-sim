from sys import argv
from simulation import simulate
from bikemodel import MeijaardModel
from graphs import plot_params

if __name__ == "__main__":
    model = MeijaardModel()
    results = simulate(model, (float(argv[1]), float(argv[2]), float(argv[3]), float(argv[4])), float(argv[5]),
                       float(argv[6]))

    plot_params('Uncontrolled Bicycle at %.2f m/s' % float(argv[6]), (results['t'], 'Time (seconds)'),
                (results['phi'], 'Phi (radians)'), (results['delta'], 'Delta (radians)'))
