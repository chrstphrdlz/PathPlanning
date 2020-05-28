mod path_finding {
    use petgraph::algo::{bellman_ford, dijkstra};
    use petgraph::graphmap::UnGraphMap;
    pub use petgraph::Graph;
    use std::borrow::Borrow;
    use std::collections::HashSet;
    use std::time::Instant;

    type Point = (i32, i32);

    fn neighbors(p: Point, x_len: i32, y_len: i32) -> Vec<Point> {
        if p.0 < 0 || p.1 < 0 || p.0 >= x_len || p.1 >= y_len {
            panic!("point ({}, {}) is out of bounds of the graph!", p.0, p.1);
        }

        (-1..2)
            .map(|dx| {
                (-1..2)
                    .filter(|&dy| !(dx == 0 && dy == 0))
                    .map(|dy| (p.0 + dx, p.1 + dy))
                    .collect::<Vec<Point>>()
            })
            .flatten()
            .filter(|&(x, y)| x >= 0 && y >= 0)
            .filter(|&(x, y)| x < x_len && y < y_len)
            .collect::<Vec<Point>>()
    }

    // Graph specified by having each tile connected to all its neighbors by edges.
    // Each edge is marked with a boolean to indicate if it blocked.
    pub fn build_empty_tile_graph(x_size: i32, y_size: i32) -> UnGraphMap<Point, i32> {
        if x_size < 1 || y_size < 1 {
            panic!(
                "Invalid values for the graph size x: {}, y: {}",
                x_size, y_size
            );
        }
        let points = (0..x_size)
            .map(|x| (0..y_size).map(move |y| (x, y)))
            .flatten()
            .collect::<Vec<Point>>();

        let mut g = UnGraphMap::<Point, i32>::new();
        for p in points {
            g.add_node(p);
            for n in neighbors(p, x_size, y_size) {
                g.add_edge(p, n, 1);
            }
        }
        return g;
    }

    fn _neighbors_3x3(p: Point) -> Vec<Point> {
        neighbors(p, 3, 3)
    }

    struct Line {
        first: Point,
        second: Point,
    }

    struct Path {
        line: Line,
        curr_x: i32,
    }

    impl Iterator for Path {
        type Item = Point;

        fn next(&mut self) -> Option<Self::Item> {
            return if self.curr_x < self.line.second.0 {
                let line_angle_to_origin = ((self.line.second.1 - self.line.first.1) as f64
                    / (self.line.second.0 - self.line.first.0) as f64)
                    .atan();
                let dx = self.curr_x - self.line.first.0;
                let dy = dx as f64 * line_angle_to_origin.sin();
                let y_at_this_point = self.line.first.1 + (dy as i32);
                let next: Point = (self.curr_x, y_at_this_point);
                self.curr_x = self.curr_x + 1;
                Option::Some(next)
            } else if self.curr_x == self.line.second.0 {
                Option::Some(self.line.second)
            } else {
                Option::None
            };
        }
    }

    fn min_distance_from_line(point: Point, line: Line) -> f64 {
        let line_angle_to_origin =
            ((line.second.1 - line.first.1) as f64 / (line.second.0 - line.first.0) as f64).atan();
        let point_angle_to_origin = (point.1 as f64 / point.0 as f64).atan();
        let distance_origin_to_point = ((point.0 * point.0 + point.1 * point.1) as f64).sqrt();
        return ((point_angle_to_origin - line_angle_to_origin).sin() * distance_origin_to_point)
            .abs();
    }

    fn collision_between(p1: Point, p2: Point, g: UnGraphMap<Point, i32>) -> bool {
        let path: Path = Path {
            line: Line {
                first: p1.clone(),
                second: p2.clone(),
            },
            curr_x: p1.0,
        };

        for step in path {
            if !g.contains_node(step) {
                return true;
            }
        }

        return false;
    }

    #[test]
    fn find_path_dfs() {
        // Approximately 1 cm descretization, 100 meters in length and width.
        for graph_size_power in (7..10) {
            let now = Instant::now();
            let base :i32= 2;
            let graph_size = base.pow(graph_size_power);
            let house: petgraph::graphmap::GraphMap<(i32, i32), i32, petgraph::Undirected> =
                build_empty_tile_graph(graph_size, graph_size);
            dijkstra(
                house.borrow(),
                (0, 0),
                Option::Some((graph_size - 1, graph_size - 1)),
                |_| 1,
            );
            println!("graph size: {}, milliseconds elapsed: {}", graph_size, now.elapsed().as_millis());
        }
    }

    #[test]
    fn neighbors_all_around() {
        let p = (1, 1);
        assert_eq!(
            vec![
                (0, 0),
                (0, 1),
                (0, 2),
                (1, 0),
                (1, 2),
                (2, 0),
                (2, 1),
                (2, 2)
            ],
            _neighbors_3x3(p)
        );
    }

    #[test]
    fn neighbors_on_x_0() {
        let only_point = (0, 1);
        assert_eq!(
            vec![(0, 0), (0, 2), (1, 0), (1, 1), (1, 2)],
            _neighbors_3x3(only_point)
        );
    }

    #[test]
    fn neighbors_on_x_boundary() {
        let only_point = (2, 1);
        assert_eq!(
            vec![(1, 0), (1, 1), (1, 2), (2, 0), (2, 2)],
            _neighbors_3x3(only_point)
        );
    }

    #[test]
    fn neighbors_on_y_0() {
        let only_point = (1, 0);
        assert_eq!(
            vec![(0, 0), (0, 1), (1, 1), (2, 0), (2, 1)],
            _neighbors_3x3(only_point)
        );
    }

    #[test]
    fn neighbors_on_y_boundary() {
        let only_point = (1, 2);
        assert_eq!(
            vec![(0, 1), (0, 2), (1, 1), (2, 1), (2, 2)],
            _neighbors_3x3(only_point)
        );
    }

    #[test]
    fn no_neighbors() {
        assert!(neighbors((0, 0), 1, 1).is_empty());
    }

    #[should_panic]
    #[test]
    fn ptx_lt_0() {
        _neighbors_3x3((-1, 0));
    }

    #[should_panic]
    #[test]
    fn pty_lt_0() {
        _neighbors_3x3((0, -1));
    }

    #[should_panic]
    #[test]
    fn ptx_gt_size() {
        _neighbors_3x3((3, 0));
    }

    #[should_panic]
    #[test]
    fn pty_gt_size() {
        _neighbors_3x3((3, 3));
    }

    #[should_panic]
    #[test]
    fn size_x_lt_1() {
        build_empty_tile_graph(1, 0);
    }

    #[should_panic]
    #[test]
    fn size_y_lt_1() {
        build_empty_tile_graph(0, 1);
    }

    #[test]
    fn test_graph_building() {
        let actual = build_empty_tile_graph(1, 2);
        assert_eq!(actual.node_count(), 2);
        assert_eq!(actual.edge_count(), 1);
        assert!(actual.contains_node((0, 0)));
        assert!(actual.contains_node((0, 1)));
        assert!(actual.contains_edge((0, 1), (0, 0)));
    }
}
