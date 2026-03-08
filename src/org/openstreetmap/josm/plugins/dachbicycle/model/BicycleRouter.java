package org.openstreetmap.josm.plugins.dachbicycle.model;

import java.util.*;
import org.openstreetmap.josm.data.osm.DataSet;
import org.openstreetmap.josm.data.osm.Node;
import org.openstreetmap.josm.data.osm.Way;

public class BicycleRouter {

    // Hilfsklasse für den Graphen
    private static class Edge {
        Node target;
        Way viaWay;
        double cost;

        Edge(Node target, Way viaWay, double cost) {
            this.target = target;
            this.viaWay = viaWay;
            this.cost = cost;
        }
    }

    // Hilfsklasse für die Priority Queue im A*
    private static class SearchNode implements Comparable<SearchNode> {
        Node current;
        SearchNode parent;
        Way reachedVia;
        double gCost; // Kosten vom Start
        double fCost; // gCost + Heuristik (geschätzte Distanz zum Ziel)

        SearchNode(Node current, SearchNode parent, Way reachedVia, double gCost, double fCost) {
            this.current = current;
            this.parent = parent;
            this.reachedVia = reachedVia;
            this.gCost = gCost;
            this.fCost = fCost;
        }

        @Override
        public int compareTo(SearchNode other) {
            return Double.compare(this.fCost, other.fCost);
        }
    }

    /**
     * Baut einen Graphen aus den aktuell in JOSM geladenen Daten
     */
    private Map<Node, List<Edge>> buildGraph(DataSet ds) {
        Map<Node, List<Edge>> graph = new HashMap<>();

        for (Way way : ds.getWays()) {
            if (!way.isUsable() || way.getNodesCount() < 2) continue;
            
            // Ignoriere Autobahnen und reine Fußwege (vereinfacht)
            String hw = way.get("highway");
            if (hw == null || hw.equals("motorway") || hw.equals("steps")) continue;

            // Fahrradwege (cycleway) sind "billiger", Hauptstraßen (primary) sind "teurer"
            double penalty = 1.0;
            if ("cycleway".equals(hw)) penalty = 0.5; // Halbe Kosten = Bevorzugt!
            if ("primary".equals(hw)) penalty = 3.0;  // Dreifache Kosten = Vermeiden!

            List<Node> nodes = way.getNodes();
            for (int i = 0; i < nodes.size() - 1; i++) {
                Node n1 = nodes.get(i);
                Node n2 = nodes.get(i + 1);
                
                if (n1.getCoor() == null || n2.getCoor() == null) continue;
                
                double dist = n1.getCoor().greatCircleDistance(n2.getCoor()) * penalty;

                graph.putIfAbsent(n1, new ArrayList<>());
                graph.putIfAbsent(n2, new ArrayList<>());

                // Graph ist bidirektional (wir ignorieren oneway für den Moment)
                graph.get(n1).add(new Edge(n2, way, dist));
                graph.get(n2).add(new Edge(n1, way, dist));
            }
        }
        return graph;
    }

    /**
     * Der eigentliche A*-Suchalgorithmus
     */
    public List<Way> findRoute(DataSet ds, Node start, Node target) {
        Map<Node, List<Edge>> graph = buildGraph(ds);
        if (!graph.containsKey(start) || !graph.containsKey(target)) return new ArrayList<>();

        PriorityQueue<SearchNode> openSet = new PriorityQueue<>();
        Map<Node, SearchNode> allNodes = new HashMap<>();

        SearchNode startNode = new SearchNode(start, null, null, 0.0, heuristic(start, target));
        openSet.add(startNode);
        allNodes.put(start, startNode);

        Set<Node> closedSet = new HashSet<>();

        while (!openSet.isEmpty()) {
            SearchNode current = openSet.poll();

            // Ziel erreicht!
            if (current.current.equals(target)) {
                return reconstructPath(current);
            }

            closedSet.add(current.current);

            for (Edge edge : graph.getOrDefault(current.current, new ArrayList<>())) {
                if (closedSet.contains(edge.target)) continue;

                double tentativeGCost = current.gCost + edge.cost;
                SearchNode neighbor = allNodes.get(edge.target);

                if (neighbor == null || tentativeGCost < neighbor.gCost) {
                    double fCost = tentativeGCost + heuristic(edge.target, target);
                    SearchNode nextNode = new SearchNode(edge.target, current, edge.viaWay, tentativeGCost, fCost);
                    
                    allNodes.put(edge.target, nextNode);
                    openSet.add(nextNode);
                }
            }
        }
        return new ArrayList<>(); // Kein Weg gefunden
    }

    private double heuristic(Node a, Node b) {
        if (a.getCoor() == null || b.getCoor() == null) return 0;
        return a.getCoor().greatCircleDistance(b.getCoor());
    }

    private List<Way> reconstructPath(SearchNode node) {
        // Da wir eine Liste von Wegen brauchen, nutzen wir ein Set, um Duplikate zu vermeiden
        // (Ein Weg besteht aus vielen Segmenten/Nodes)
        Set<Way> pathWays = new LinkedHashSet<>(); 
        SearchNode curr = node;
        while (curr.parent != null) {
            pathWays.add(curr.reachedVia);
            curr = curr.parent;
        }
        List<Way> result = new ArrayList<>(pathWays);
        Collections.reverse(result);
        return result;
    }
}