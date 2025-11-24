import rclpy
from rclpy.node import Node
import py_trees
import py_trees.common
from py_trees.blackboard import Client
import time

from py_trees_meet_groot import groot_xml

from bt_bumpgo.bumpgo_bt import MoveForward, CheckBump, BackOff, Turn


'''
Para que esto funcione, hay que editar /home/roi/.local/lib/python3.12/site-packages/py_trees_meet_groot/groot_xml.py:

elif str(e.nodeName) == "Fallback":
    nodes = parse_BehaviourTree(e, dict_bh, decorators)
    # Fallback "normal" → memory=True
    sel = py_trees.composites.Selector(name="selector", memory=True)
    sel.add_children(nodes)
    ret.append(sel)

elif str(e.nodeName) == "ReactiveFallback":
    nodes = parse_BehaviourTree(e, dict_bh, decorators)
    # ReactiveFallback → memory=False
    sel = py_trees.composites.Selector(name="selector", memory=False)
    sel.add_children(nodes)
    ret.append(sel)

elif str(e.nodeName) == "Sequence":
    nodes = parse_BehaviourTree(e, dict_bh, decorators)
    # Sequence "normal" → memory=True (equivalente a BT con memoria)
    seq = py_trees.composites.Sequence(name="sequence", memory=True)
    seq.add_children(nodes)
    ret.append(seq)

elif str(e.nodeName) == "ReactiveSequence":
    nodes = parse_BehaviourTree(e, dict_bh, decorators)
    # ReactiveSequence → memory=False
    seq = py_trees.composites.Sequence(name="sequence", memory=False)
    seq.add_children(nodes)
    ret.append(seq)    
'''


def main(args=None):
    rclpy.init(args=args)
    ros_node = Node('bump_go')

    blackboard = Client(name="global_blackboard")
    blackboard.register_key(key="node", access=py_trees.common.Access.WRITE)
    blackboard.node = ros_node

    move_forward_bh = MoveForward("MoveForward")
    check_bump_bh   = CheckBump("CheckBump")
    back_off_bh     = BackOff("BackOff")
    turn_bh         = Turn("Turn")

    custom_behaviors = [
        move_forward_bh,
        check_bump_bh,
        back_off_bh,
        turn_bh,
    ]

    xml_path = "/home/roi/UNI/docencia/repos/asr-py/src/bt_bumpgo/bt_xml/bumpgo.xml"

    print(f"--> ATTEMPTING TO LOAD: {xml_path}")
    print("Behaviours disponibles:", [b.name for b in custom_behaviors])

    # Cargar el árbol desde el XML de Groot
    root = groot_xml.load(xml_path, behaviors=custom_behaviors)

    print("Root cargado:", root)

    # Setup de todo el árbol
    root.setup_with_descendants()

    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
            root.tick_once()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
