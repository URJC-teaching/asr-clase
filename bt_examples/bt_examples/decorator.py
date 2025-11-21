
import py_trees
import py_trees.behaviour
import py_trees.composites
import py_trees.common
import random

class RandomNumber(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)        

    def update(self):
        number = random.randint(0, 10)
        print(f'[{self.name}] Generated number: {number}.')
        if number > 9:
            print(f'[{self.name}] Number {number} is greater than 8. Succeeding.')
            return py_trees.common.Status.SUCCESS
        else:
            print(f'Number {number} is not greater than 8. Failing.')
            return py_trees.common.Status.FAILURE


def create_root():
    root = py_trees.composites.Sequence("BT example", memory=True)

    r1 = RandomNumber("RandomNumber 1")
    r2 = RandomNumber("RandomNumber 2")
    # Check decorators documentation for more options: https://py-trees.readthedocs.io/en/devel/decorators.html
    failure_is_success = py_trees.decorators.FailureIsSuccess("FailureIsSuccess", child=r1)
    retry = py_trees.decorators.Retry("Retry", child=r2, num_failures=float('inf'))
    root.add_child(failure_is_success)
    root.add_child(retry)
    
    return root

def main():
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    root = create_root()

    root.setup_with_descendants()

    while True:
        root.tick_once()
        print(f'Root status: {root.status}')
        if root.status == py_trees.common.Status.SUCCESS or root.status == py_trees.common.Status.FAILURE:
            print(f'Behavior Tree finished with status: {root.status}')
            break

if __name__ == '__main__':
    main()