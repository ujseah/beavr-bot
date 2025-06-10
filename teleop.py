import hydra
from beavr.components import TeleOperator

def run_teleop(configs):
    """Run the teleoperation system with given configs"""
    teleop = TeleOperator(configs)
    processes = teleop.get_processes()

    for process in processes:
        process.start()

    for process in processes:
        process.join()

@hydra.main(version_base = '1.2', config_path = 'configs', config_name = 'teleop')
def main(configs):
    run_teleop(configs)

if __name__ == '__main__':
    main()