import numpy as np

class RRTNode:

    #######################################
    # Attributes
    #######################################

    @property
    def q(self):
        return self._q

    @property
    def parent(self):
        return self._parent

    @property
    def children(self):
        return self._children

    @property
    def value_node(self):
        return self._value_node

    @property
    def index(self):
        return self._index


    #######################################
    # Constructor
    #######################################

    def __init__(self, q, value_node, index):
        self._q = q
        self._parent = None
        self._children = []
        self._value_node = value_node
        self._index = index

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "RRT Node with id " + str(self.index) + ": \n\n"
        string += "Value: "+str(round(self.value_node,5)) + " \n"
        string += "Parent node: "+(str(self.parent.index) if (not self.parent is None) else "root node")+ " \n"
        string += "Children nodes: "+str([node.index for node in self.children])+ " \n"
        string += "Configuration: "+str(self.q.T)

        return string

    #######################################
    # Methods
    #######################################

    def set_parent(self, node_parent):
        if not self._parent is None:
            self._parent._children.remove(self)

        self._parent = node_parent
        node_parent._children.append(self)

    def add_children(self, node_child):
        node_child.set_parent(self)

    def path_to_root(self):
        current_node = self
        list_node=[]
        while not current_node is None:
            list_node.append(current_node)
            current_node = current_node.parent

        list_node.reverse()

        return list_node

class RRTEdge:

    #######################################
    # Attributes
    #######################################

    @property
    def from_node(self):
        return self._from_node

    @property
    def to_node(self):
        return self._to_node

    @property
    def weight(self):
        return self._weight

    #######################################
    # Constructor
    #######################################

    def __init__(self, from_node, to_node, weight=0):
        self._from_node = from_node
        self._to_node = to_node
        self._weight = weight

    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "RRT Edge from node " + str(self.from_node.index) + " to "+ str(self.to_node.index) +": \n\n"
        string += "Weight: "+str(round(self.weight,5)) + "\n"
        return string


class RRTTree:

    #######################################
    # Attributes
    #######################################

    @property
    def root_node(self):
        return self._root_node

    @property
    def list_node(self):
        return self._list_node

    @property
    def list_edge(self):
        return self._list_edge

    @property
    def fun_eval_node(self):
        return self._fun_eval_node

    @property
    def fun_dist_node(self):
        return self._fun_dist_node

    @property
    def fun_free(self):
        return self._fun_free

    @property
    def min_dist(self):
        return self._min_dist

    @property
    def step_size(self):
        return self._step_size

    @property
    def q_inf(self):
        return self._q_inf

    @property
    def q_sup(self):
        return self._q_sup

    #######################################
    # Constructor
    #######################################

    def __init__(self, root_q, fun_eval_node, fun_free, q_inf, q_sup,
                 fun_dist_node = lambda q1, q2 : np.linalg.norm(q1-q2), min_dist=0.01, step_size=0.4):

        self._root_node = RRTNode(root_q, fun_eval_node(root_q), 0)
        self._fun_eval_node = fun_eval_node
        self._fun_free = fun_free
        self._fun_dist_node = fun_dist_node
        self._list_node = [self._root_node]
        self._list_edge = []
        self._kd_tree = []
        self._min_dist = min_dist
        self._q_inf = q_inf
        self._q_sup = q_sup
        self._step_size = step_size




    #######################################
    # Std. Print
    #######################################

    def __repr__(self):

        string = "RRT Tree with" + str(len(self.list_node)) + " nodes."
        return string

    #######################################
    # Methods
    #######################################

    def nearest_neighbors(self, q_query, k=1):

        list_dist=[]
        for i in range(len(self.list_node)):
            list_dist.append(self.fun_dist_node(q_query, self.list_node[i].q))

        ind = np.argsort(list_dist)

        list_node = []
        for i in range(min(len(list_dist),k)):
            list_node.append(self.list_node[ind[i]])

        return list_node if k > 1 else list_node[-1]

    def sample_free_config(self):


        n = np.shape(self.root_node.q)[0]
        ok = False
        while not ok:
            q_rand = np.matrix(np.zeros( (n,1) ))
            for i in range(n):
                q_rand[i,0] = self.q_inf[i,0] + (self.q_sup[i,0]-self.q_inf[i,0])*np.random.uniform(0,1)

            ok = self.fun_free(q_rand)

        return q_rand

    def config_inside_bounds(self, q):

        inside = True
        n = np.shape(self.root_node.q)[0]
        k=0

        while inside and k < n:
            inside = self.q_sup[k, 0] >= q[k, 0] >= self.q_inf[k, 0]
            k+=1

        return inside

    def add_node_tree(self, k=1):


        if k==1:
            ok = False

            while not ok:

                q_rand = self.sample_free_config()
                node_prox = self.nearest_neighbors(q_rand)
                q_prox = node_prox.q
                dir = (q_rand - q_prox) / (0.0001 + np.linalg.norm(q_rand - q_prox))
                q_target = q_prox + np.random.uniform(0.2 * self.step_size, self.step_size) * dir

                ok = self.config_inside_bounds(q_target) and self.is_path_free(q_prox, q_target)

                if ok:
                    new_node = RRTNode(q_target, self.fun_eval_node(q_target), len(self.list_node))
                    new_node.set_parent(node_prox)
                    new_edge = RRTEdge(node_prox, new_node)
                    self.list_node.append(new_node)
                    self.list_edge.append(new_edge)
        else:
            for i in range(k):
                self.add_node_tree(1)

    def is_path_free(self, q_init, q_end):

        if self.fun_free( (q_init + q_end)/2 ):
            if self.fun_dist_node(q_init, q_end) < self.min_dist:
                return True
            else:
                return self.is_path_free(q_init, (q_init + q_end) / 2) and self.is_path_free((q_init + q_end) / 2,
                                                                                             q_end)
        else:
            return False

    def node_smallest_value(self):

        min_node = self.root_node

        for node in self.list_node:
            if node.value_node < min_node.value_node:
                min_node = node

        return min_node




