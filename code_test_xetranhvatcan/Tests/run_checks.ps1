$ErrorActionPreference = 'Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $cases = @(
        @{Name='board_drive'; Sources=@('Tests/test_board_drive.c','Src/board_drive.c','Tests/fakes/board_gpio.c'); Hal=$true},
        @{Name='motion'; Sources=@('Tests/test_motion.c','Src/motion.c','Src/board_drive.c','Tests/fakes/board_gpio.c'); Hal=$true},
        @{Name='teach_drive'; Sources=@('Tests/test_teach_drive.c'); Hal=$true},
        @{Name='motor_bench'; Sources=@('Tests/test_motor_bench.c','Src/motor_bench.c','Src/board_drive.c','Tests/fakes/board_gpio.c'); Hal=$true},
        @{Name='front_clearance'; Sources=@('Tests/test_front_clearance.c','Src/front_clearance.c')},
        @{Name='grid_map'; Sources=@('Tests/test_grid_map.c','Src/grid_map.c')},
        @{Name='navigation'; Sources=@('Tests/test_navigation.c','Src/navigation.c','Src/grid_map.c'); Hal=$true},
        @{Name='tof_offset'; Sources=@('Tests/test_tof_offset.c')},
        @{Name='text_format'; Sources=@('Tests/test_text_format.c')},
        @{Name='vl53l0x'; Sources=@('Tests/test_vl53l0x.c','Src/vl53l0x.c'); Hal=$true}
    )
    foreach ($case in $cases) {
        $arguments = @('-std=c11','-Wall','-Wextra','-Werror','-IInc')
        if ($case.Hal) { $arguments += '-ITests/fakes' }
        $exe = "build/test_$($case.Name).exe"
        & gcc @arguments @($case.Sources) -lm -o $exe
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case.Name)" }
        & (Join-Path (Get-Location) $exe)
        if ($LASTEXITCODE -ne 0) { throw "Check failed: $($case.Name)" }
    }
    & gcc -std=c11 -Wall -Wextra -Werror -IInc -ITests/fakes Tests/sim_course.c Src/grid_map.c -lm -o build/sim_course.exe
    if ($LASTEXITCODE -ne 0) { throw 'Compile failed: sim_course' }
    foreach ($scenario in @('nominal','t9','front_open','front_no_echo','t9_no_echo','invalid_center','extra')) {
        & (Join-Path (Get-Location) 'build/sim_course.exe') trace $scenario | Out-Null
        if ($LASTEXITCODE -ne 0) { throw "Check failed: sim_course $scenario" }
    }
} finally { Pop-Location }
