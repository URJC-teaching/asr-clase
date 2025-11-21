
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
        if number > 5:
            print(f'[{self.name}] Number {number} is greater than 5. Succeeding.')
            return py_trees.common.Status.SUCCESS
        else:
            print(f'Number {number} is not greater than 5. Failing.')
            return py_trees.common.Status.FAILURE


def create_root():
    root = py_trees.composites.Sequence("BT example", memory=True)
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

    # node = py_trees_ros.trees.BehaviourTree(root)
    # root.setup(timeout=15) 

    # # Tick tree till we get SUCCESS or FAILURE
    # while True:
    #     status = node.tree.tick_tock(500)
    #     if status != py_trees.common.Status.RUNNING:
    #         break

if __name__ == '__main__':
    main()