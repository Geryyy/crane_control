# crane_control

First-slice shell for W04 and W07. It intentionally installs no C++ headers,
libraries, or runtime nodes. Its deterministic seam fixtures are private to
the package tests and only exercise the installed `crane_model` mock.
Controller plugins, valve inversion, PI gains, hardware access, and a
passive-state algorithm remain out of scope. No `epsilon_crane_hardware`,
`ax_comp_lib`, or retained controller is linked.
