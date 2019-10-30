from sys import argv
from .simulation import simulate
from .bikemodel import MeijaardModel

if __name__ == "__main__":
    model = MeijaardModel()
    results = simulate(model, (argv[1], argv[2], argv[3], argv[4]), argv[5], argv[6])
