import java.util.*;

public class DijkstraShortestPath {
    private static final int INFINITY = Integer.MAX_VALUE;

    private static class Node {
        private String name;
        private List<Edge> edges;

        public Node(String name) {
            this.name = name;
            this.edges = new ArrayList<>();
        }

        public void addEdge(Node destination, int weight) {
            edges.add(new Edge(destination, weight));
        }

        public List<Edge> getEdges() {
            return edges;
        }
    }

    private static class Edge {
        private Node destination;
        private int weight;

        public Edge(Node destination, int weight) {
            this.destination = destination;
            this.weight = weight;
        }

        public Node getDestination() {
            return destination;
        }

        public int getWeight() {
            return weight;
        }
    }

    public static void main(String[] args) {
        if (args.length != 2) {
            System.out.println("Usage: java DijkstraShortestPath <source> <destination>");
            return;
        }

        // Create the graph
        Node A = new Node("A");
        Node B = new Node("B");
        Node C = new Node("C");
        Node D = new Node("D");
        Node E = new Node("E");
        Node F = new Node("F");
        Node G = new Node("G");
        Node H = new Node("H");
        Node I = new Node("I");
        Node J = new Node("J");
        Node K = new Node("K");
        Node L = new Node("L");
        Node M = new Node("M");
        Node N = new Node("N");
        Node O = new Node("O");
        Node P = new Node("P");
        Node Q = new Node("Q");
        Node R = new Node("R");
        Node S = new Node("S");
        Node T = new Node("T");
        Node U = new Node("U");
        Node V = new Node("V");
        Node W = new Node("W");
        Node X = new Node("X");
        Node Y = new Node("Y");
        Node Z = new Node("Z");

        A.addEdge(B, 3);
        A.addEdge(C, 4);
        B.addEdge(D, 2);
        B.addEdge(E, 5);
        C.addEdge(D, 5);
        C.addEdge(F, 6);
        D.addEdge(H, 3);
        D.addEdge(I, 4);
        E.addEdge(J, 2);
        E.addEdge(K, 3);
        F.addEdge(G, 4);
        F.addEdge(L, 5);
        G.addEdge(H, 2);
        G.addEdge(L, 5);
        H.addEdge(M, 3);
        I.addEdge(M, 3);
        J.addEdge(K, 2);
        J.addEdge(N, 4);
        K.addEdge(N, 4);
        L.addEdge(O, 4);
        L.addEdge(P, 3);
        L.addEdge(M, 2);
        M.addEdge(Q, 2);
        M.addEdge(R, 5);
        M.addEdge(N, 6);
        N.addEdge(T, 1);
        N.addEdge(S, 2);
        O.addEdge(U, 3);
        P.addEdge(U, 3);
        Q.addEdge(V, 4);
        Q.addEdge(R, 1);
        R.addEdge(V, 4);
        S.addEdge(T, 3);
        S.addEdge(W ,2);
	T.addEdge(W, 2);
        U.addEdge(X, 2);
        U.addEdge(V, 3);
        V.addEdge(X, 4);
        V.addEdge(Y, 1);
        V.addEdge(W, 5);
        W.addEdge(Y, 3);
        X.addEdge(Z, 4);
        X.addEdge(Y, 2);
        Y.addEdge(Z, 3);

        // Retrieve the source and destination nodes
        Node source = getNode(args[0], A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z);
        Node destination = getNode(args[1], A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z);

        if (source == null || destination == null) {
            System.out.println("Invalid source or destination node.");
            return;
        }

        // Run Dijkstra's algorithm
        Map<Node, Integer> distances = new HashMap<>();
        Map<Node, Node> previousNodes = new HashMap<>();
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(distances::get));
        Set<Node> visited = new HashSet<>();

        for (Node node : Arrays.asList(A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, U, V, W, X, Y, Z)) {
            distances.put(node, INFINITY);
            previousNodes.put(node, null);
        }

        distances.put(source, 0);
        priorityQueue.add(source);

        while (!priorityQueue.isEmpty()) {
            Node current = priorityQueue.poll();
            visited.add(current);

            for (Edge edge : current.getEdges()) {
                Node neighbor = edge.getDestination();
                int newDistance = distances.get(current) + edge.getWeight();

                if (!visited.contains(neighbor) && newDistance < distances.get(neighbor)) {
                    distances.put(neighbor, newDistance);
                    previousNodes.put(neighbor, current);
                    priorityQueue.add(neighbor);
                }
            }
        }

        // Retrieve the shortest path
        List<Node> shortestPath = new ArrayList<>();
        Node node = destination;
        while (node != null) {
            shortestPath.add(0, node);
            node = previousNodes.get(node);
        }

        // Output the results
        if (distances.get(destination) == INFINITY) {
            System.out.println("There is no path between " + source.name + " and " + destination.name + ".");
        } else {
            System.out.println("Shortest path between " + source.name + " and " + destination.name + ":");
            System.out.print("Shortest path: ");
            for (int i = 0; i < shortestPath.size(); i++) {
                System.out.print(shortestPath.get(i).name);
                if (i < shortestPath.size() - 1) {
                    System.out.print("-");
                }
            }
            System.out.println();
            System.out.println("Distance: " + distances.get(destination));
        }
    }

    private static Node getNode(String name, Node... nodes) {
        for (Node node : nodes) {
            if (node.name.equals(name)) {
                return node;
            }
        }
        return null;
    }
}
