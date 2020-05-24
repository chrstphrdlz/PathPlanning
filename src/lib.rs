mod path_finding {
    use petgraph::graphmap::UnGraphMap;
    pub use petgraph::Graph;

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
