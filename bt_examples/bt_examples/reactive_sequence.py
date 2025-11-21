
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
        if number < 5:
            print(f'[{self.name}] Number {number} is less than 5. Succeeding.')
            return py_trees.common.Status.SUCCESS
        else:
            print(f'[{self.name}] Number {number} is not less than 5. Running.')
            return py_trees.common.Status.RUNNING
        
class JustPrints(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)        

    def update(self):
        print(f'[{self.name}] Hi there! I am just logging this message.')
        return py_trees.common.Status.SUCCESS


def create_root():
    root = py_trees.composites.Sequence("BT example", memory=False)
    printer_action_1 = JustPrints("Printer_1")
    random_action = RandomNumber("RandomNumber")
    printer_action_2 = JustPrints("Printer_2")
    root.add_child(printer_action_1)
    root.add_child(random_action)
    root.add_child(printer_action_2)
    
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