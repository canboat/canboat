"""
Module that contains the command line app.

Why does this file exist, and why not put this in __main__?

  You might be tempted to import things from __main__ later, but that will cause
  problems: the code will get executed twice:

  - When you run `python -mdbc_exporter` python will execute
    ``__main__.py`` as a script. That means there won't be any
    ``dbc_exporter.__main__`` in ``sys.modules``.
  - When you import __main__ it will get executed again (as a module) because
    there's no ``dbc_exporter.__main__`` in ``sys.modules``.

  Also see (1) from http://click.pocoo.org/5/setuptools/#setuptools-integration
"""
import argparse
import logging

import cantools

from dbc_exporter.input.canboat import CanboatReader


def parse_arguments(args):
    """
    Parse command line arguments.

    :return: Namespace with arguments.
    """
    parser = argparse.ArgumentParser(
        description="Canboat DBC Exporter converts canboat json files into the DBC format"
    )

    parser.add_argument("--verbose", "-v", action="count", default=1)

    parser.add_argument("input_file", metavar="INPUT", help="Canboat json input file name")

    parser.add_argument("output_file", metavar="OUTPUT", help="DBC output file name")

    return parser.parse_args(args)


def main(args=None):
    args = parse_arguments(args)

    # set default logging level to WARNING
    args.verbose = 40 - (10 * args.verbose) if args.verbose > 0 else 0

    logging.basicConfig(
        level=args.verbose,
        format="{asctime}.{msecs:0<3.0f} {name} {levelname}: {message}",
        style="{",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    logging.debug("Constructing the message database")

    can_database = CanboatReader(open(args.input_file, "r")).get_database()

    logging.debug("Number of read messages: %d", len(can_database.messages))

    logging.debug("Exporting the database to DBC format")

    cantools.database.dump_file(can_database, args.output_file, database_format="dbc")


if __name__ == "__main__":
    main()
