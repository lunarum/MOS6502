# ProcessorTests

This repository [Thomas Harte - ProcessorTests](https://github.com/TomHarte/ProcessorTests) contains tests for a variety of processors, provided as an aid to reimplementation.

Each test:
* requires execution of only a single instruction; and
* provides full processor and memory state before and after.

Tests are randomly generated, in substantial volume.

## Methodology

To generate each test set, an implementation is used that:
* conforms to all available documentation, official and third-party;
* passes all other published test sets; and
* has been verified by usage in an emulated machine.

In addition to the standard Git history, test sets are manually versioned to permit for potential future breaking changes in JSON format.

Please report any discrepancies uncovered, either as an issue or via a correcting pull request.

## Other Test Sets

For similar test sets from other, see:
* https://github.com/adtennant/gameboy-test-data for the Game Boy; and
* https://github.com/raddad772/jsmoo/tree/main/misc/tests/GeneratedTests for the Z80, and a separate instance of the SPC 700 tests contributed here.

# 6502

10,000 tests are provided per opcode. All tests assume a full 64kb of RAM is mapped to the processor.

Sample test:

	{
		"name": "b1 28 b5",
		"initial": {
			"pc": 59082,
			"s": 39,
			"a": 57,
			"x": 33,
			"y": 174,
			"p": 96,
			"ram": [
				[59082, 177],
				[59083, 40],
				[59084, 181],
				[40, 160],
				[41, 233],
				[59982, 119]
			]
		},
		"final": {
			"pc": 59084,
			"s": 39,
			"a": 119,
			"x": 33,
			"y": 174,
			"p": 96,
			"ram": [
				[40, 160],
				[41, 233],
				[59082, 177],
				[59083, 40],
				[59084, 181],
				[59982, 119]
			]
		},
		"cycles": [
			[59082, 177, "read"],
			[59083, 40, "read"],
			[40, 160, "read"],
			[41, 233, "read"],
			[59083, 40, "read"],
			[59982, 119, "read"]
		]
	}

`name` is provided for human consumption and has no formal meaning.

`initial` is the initial state of the processor; `ram` contains a list of values to store in memory prior to execution, each one in the form `[address, value]`.

`final` is the state of the processor and relevant memory contents after execution.

`cycles` provides a cycle-by-cycle breakdown of bus activity in the form `[address, value, type]` where `type` is either `read` or `write`.