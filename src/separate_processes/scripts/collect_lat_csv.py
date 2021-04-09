import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--directory', type=str, help='relative path to parent directory containing dumped csvs.')
    parser.add_argument('--nodes', nargs="+", default=[3, 23, 1], help="""Number of nodes to process. Pass it as follows:
                                                                          <start_nodes> <end_nodes> <step_size>.""")
    parser.add_argument('--rmw', type=str, default="fastrtps", help="Choose RMW. Allowed values: cyclone, fastrtps, connext")
    parser.add_argument('--f', type=int, default=1, help="Publisher Frequency in Hz.")
    parser.add_argument('--msg-size', type=str, default="128b", help="Size of the message.")
    parser.add_argument('--reliability', type=str, default="best_effort", help="Reliability, best_effort or reliable")
    args = parser.parse_args()

