$ErrorActionPreference = 'Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $cases = @(
        @{Name='motion'; Sources=@('Tests/test_motion.c','Src/motion.c'); Hal=$true},
        @{Name='front_clearance'; Sources=@('Tests/test_front_clearance.c','Src/front_clearance.c')},
        @{Name='grid_map'; Sources=@('Tests/test_grid_map.c','Src/grid_map.c')},
        @{Name='navigation'; Sources=@('Tests/test_navigation.c','Src/navigation.c','Src/grid_map.c'); Hal=$true},
        @{Name='tof_offset'; Sources=@('Tests/test_tof_offset.c')},
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
} finally { Pop-Location }
