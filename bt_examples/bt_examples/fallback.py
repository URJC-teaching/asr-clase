
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
        if number > 3:
            print(f'[{self.name}] Number {number} is greater than 3. Succeeding.')
            return py_trees.common.Status.SUCCESS
        else:
            print(f'[{self.name}] Number {number} is not greater than 3. Failing.')
            return py_trees.common.Status.FAILURE


def create_root():
    root = py_trees.composites.Selector("BT example", memory=True)
    for i in range(3):
        action = RandomNumber(f"RandomNumber {i+1}")
        root.add_child(action)
    
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