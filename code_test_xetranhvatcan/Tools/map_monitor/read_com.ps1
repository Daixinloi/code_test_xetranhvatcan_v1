param(
    [Parameter(Mandatory)][string]$Port,
    [Parameter(Mandatory)][int]$OwnerPid
)

$serial = [IO.Ports.SerialPort]::new($Port, 9600)
$serial.WriteTimeout = 700
try {
    $owner = [Diagnostics.Process]::GetProcessById($OwnerPid)
    $serial.Open()
    [Console]::WriteLine('@OPEN')
    $inputPipe = [Console]::OpenStandardInput()
    $commandByte = [byte[]]::new(1)
    $inputTask = $inputPipe.ReadAsync($commandByte, 0, 1)
    $pending = ''
    while (!$owner.HasExited) {
        if ($inputTask.IsCompleted) {
            if ($inputTask.GetAwaiter().GetResult() -eq 0) { break }
            $request = [string][char]$commandByte[0]
            if ($request -cmatch '^[wsadupxWAD]$') {
                $serial.Write($request)
                [Console]::WriteLine("@SENT $request")
            }
            $inputTask = $inputPipe.ReadAsync($commandByte, 0, 1)
        }
        $pending += $serial.ReadExisting()
        $lines = $pending -split "`n"
        for ($i = 0; $i -lt $lines.Count - 1; $i++) {
            [Console]::WriteLine($lines[$i].TrimEnd("`r"))
        }
        $pending = $lines[-1]
        if ($pending.Length -gt 1024) { $pending = '' }
        Start-Sleep -Milliseconds 15
    }
} catch {
    [Console]::WriteLine('@ERROR ' + $_.Exception.GetBaseException().Message)
} finally {
    $serial.Dispose()
}
