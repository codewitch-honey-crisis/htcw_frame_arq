using Htcw;

using System.Diagnostics;
using System.Globalization;
using System.Runtime.Versioning;
using System.Text;
using System.Threading;
namespace SerialFrameDemo;

[SupportedOSPlatform("windows")]
internal class Program
{
    static int _gpioNum = -1;
    static bool _waiting = false;

    // Reply watchdog. Bounds how long the console waits for a response frame.
    // Must exceed the transport's own request give-up window
    // (AckTimeout*(MaxRetries+1), ~6s at the defaults) plus the return trip, so a
    // failed *send* surfaces as a FrameError first and this only ever catches a
    // lost *response*. 
    const long ReplyTimeoutMs = 10000;
    static long _waitDeadline;

    static void BeginWaiting()
    {
        _waitDeadline = Environment.TickCount64 + ReplyTimeoutMs;
        _waiting = true;
        Thread.MemoryBarrier();
    }

    static void Main(string[] args)
    {
        if (args.Length == 0) throw new ArgumentException("A serial port must be specified.");
        var session = new EspSerialSession(args[0], true); // ack timeout defaults to 1000ms, 5 retries
        var buffer = new byte[InterfaceMaxSize.Value];
        Console.Error.WriteLine($"Connected to {args[0]}");
        session.FrameReceived += Session_FrameReceived;
        session.FrameError += Session_FrameError;
        session.Open();

        while (session.IsOpen)
        {
            while (_waiting)
            {
                if (Environment.TickCount64 >= _waitDeadline)
                {
                    Console.Error.WriteLine("Timed out waiting for a response");
                    _waiting = false;
                    Thread.MemoryBarrier();
                    break;
                }
                Thread.Sleep(100);
            }

            Console.Write(">");
            var cmd = Console.ReadLine();
            if (cmd == null)
            {
                continue;
            }
            var sa = cmd.ToUpperInvariant().Split(' ');
            switch (sa[0])
            {
                case "LOG":
                    Console.WriteLine(Encoding.UTF8.GetString(session.GetNextLogData()));
                    break;
                case "RANDOM":
                    HandleRandom(session, buffer, sa);
                    break;
                case "GPIO":
                    HandleGpio(session, buffer, sa);
                    break;
                case "HELP":
                case "?":
                    PrintHelp();
                    break;
                case "QUIT":
                    session.Close();
                    break;
                default:
                    Console.Error.WriteLine("Unrecognized command. Type HELP or ? for available commands");
                    Console.Error.WriteLine();
                    break;
            }
        }
        Console.Error.WriteLine($"Disconnected");
    }

    static void HandleRandom(EspSerialSession session, byte[] buffer, string[] sa)
    {
        if (sa.Length > 2)
        {
            Console.Error.WriteLine("Too many arguments");
            return;
        }
        var count = 1U;
        if (sa.Length == 2 &&
            !uint.TryParse(sa[1], CultureInfo.InvariantCulture.NumberFormat, out count))
        {
            Console.Error.WriteLine("The first argument must be a number");
            return;
        }
        if (count > 1024)
        {
            Console.Error.WriteLine("Overflow. Count must be <= 1024");
            return;
        }

        var rng = new STRngMessage { Count = count };
        if (rng.TryWrite(buffer, out int bytesWritten))
        {
            BeginWaiting();
            session.Send((byte)STMessageCommand.CmdRng, buffer.AsSpan(0, bytesWritten));
        }
    }

    static void HandleGpio(EspSerialSession session, byte[] buffer, string[] sa)
    {
        byte b;
        int bytesWritten;
        switch (sa.Length)
        {
            case 0:
            case 1:
                Console.Error.WriteLine("Not enough arguments");
                break;
            case 2:
                if (!byte.TryParse(sa[1], CultureInfo.InvariantCulture.NumberFormat, out b))
                {
                    Console.Error.WriteLine("The first argument must be a number");
                    break;
                }
                _gpioNum = b;
                var gpioGet = new STGpioGetMessage();
                gpioGet.Mask = unchecked((ulong)(1UL << b));
                if (gpioGet.TryWrite(buffer, out bytesWritten))
                {
                    BeginWaiting();
                    session.Send((byte)STMessageCommand.CmdGpioGet, buffer.AsSpan(0, bytesWritten));
                }
                break;
            case 3:
                {
                    if (!byte.TryParse(sa[1], CultureInfo.InvariantCulture.NumberFormat, out b))
                    {
                        Console.Error.WriteLine("The first argument must be a number");
                        break;
                    }
                    var mode = false;
                    var modeKind = STGpioMode.ModeInput;
                    if (sa[2] != "ON" && sa[2] != "OFF")
                    {
                        if (sa[2] == "INPUT")
                        {
                            mode = true;
                            modeKind = STGpioMode.ModeInput;
                        }
                        else if (sa[2] == "OUTPUT")
                        {
                            mode = true;
                            modeKind = STGpioMode.ModeOutput;
                        }
                        else
                        {
                            Console.Error.WriteLine("The second argument must be on, off, input or output");
                            break;
                        }
                    }
                    if (!mode)
                    {
                        var gpioSet = new STGpioSetMessage();
                        gpioSet.Mask = unchecked((ulong)(1UL << b));
                        if (sa[2] == "ON")
                        {
                            gpioSet.Values = unchecked((ulong)(1UL << b));
                        }
                        if (gpioSet.TryWrite(buffer, out bytesWritten))
                        {
                            session.Send((byte)STMessageCommand.CmdGpioSet, buffer.AsSpan(0, bytesWritten));
                        }
                    }
                    else
                    {
                        var gpioMode = new STGpioModeMessage();
                        gpioMode.Gpio = b;
                        Debug.WriteLine($"Gpio mode set for {b}");
                        gpioMode.Mode = modeKind;
                        if (gpioMode.TryWrite(buffer, out bytesWritten))
                        {
                            session.Send((byte)STMessageCommand.CmdGpioMode, buffer.AsSpan(0, bytesWritten));
                        }
                    }
                }
                break;
            case 4:
                {
                    if (!byte.TryParse(sa[1], CultureInfo.InvariantCulture.NumberFormat, out b))
                    {
                        Console.Error.WriteLine("The first argument must be a number");
                        break;
                    }

                    if (sa[2] != "INPUT" && sa[2] != "OUTPUT")
                    {
                        Console.Error.WriteLine("The third argument is only valid when the second is input or output");
                        break;
                    }
                    var modeKind = STGpioMode.ModeInput;
                    if (sa[2] == "INPUT")
                    {
                        if (sa[3] == "PULLUP")
                        {
                            modeKind = STGpioMode.ModeInputPullup;
                        }
                        else if (sa[3] == "PULLDOWN")
                        {
                            modeKind = STGpioMode.ModeInputPulldown;
                        }
                        else
                        {
                            Console.Error.WriteLine("The third argument must be pullup or pulldown when specified if the second is input");
                            break;
                        }
                    }
                    else if (sa[2] == "OUTPUT")
                    {
                        if (sa[3] == "OD")
                        {
                            modeKind = STGpioMode.ModeOutputOpenDrain;
                        }
                        else
                        {
                            Console.Error.WriteLine("The third argument must be OD when specified if the second is output");
                            break;
                        }
                    }
                    else
                    {
                        Console.Error.WriteLine("The second argument must be on, off, input or output");
                        break;
                    }

                    var gpioMode = new STGpioModeMessage();
                    gpioMode.Gpio = b;
                    gpioMode.Mode = modeKind;
                    if (gpioMode.TryWrite(buffer, out bytesWritten))
                    {
                        session.Send((byte)STMessageCommand.CmdGpioMode, buffer.AsSpan(0, bytesWritten));
                    }
                }
                break;
            default:
                Console.Error.WriteLine("Too many arguments");
                break;
        }
    }

    static void PrintHelp()
    {
        Console.Error.WriteLine("LOG      Gets the most recent log entries since the last time log was used");
        Console.Error.WriteLine();
        Console.Error.WriteLine("RANDOM   Gets a value from the ESP32 hardware RNG");
        Console.Error.WriteLine("    RANDOM <count> Gets multiple values from the ESP32 hardware RNG");
        Console.Error.WriteLine();
        Console.Error.WriteLine("GPIO     Gets or sets the GPIO level and mode");
        Console.Error.WriteLine("    GPIO <pin number> retrieves the current level");
        Console.Error.WriteLine("    GPIO <pin number> ON sets the level high");
        Console.Error.WriteLine("    GPIO <pin number> OFF sets the level low");
        Console.Error.WriteLine("    GPIO <pin number> OUTPUT sets the mode to ouput");
        Console.Error.WriteLine("    GPIO <pin number> OUTPUT OD sets the mode to ouput open drain");
        Console.Error.WriteLine("    GPIO <pin number> INPUT sets the mode to input floating");
        Console.Error.WriteLine("    GPIO <pin number> INPUT PULLUP sets the mode to input w/ pullup");
        Console.Error.WriteLine("    GPIO <pin number> INPUT PULLDOWN sets the mode to input w/ pulldown");
        Console.Error.WriteLine();
        Console.Error.WriteLine("QUIT\tExits the application");
        Console.Error.WriteLine();
    }

    private static void Session_FrameError(object? sender, FrameErrorEventArgs e)
    {
        // Repurposed: fires only when the transport gives up after exhausting retries,
        // i.e. the command never got an ACK. Release the console so it doesn't hang
        // waiting for a response that will never arrive.
        Console.Error.WriteLine($"Command 0x{e.Command:X2} could not be delivered after {e.Attempts} attempt(s)");
        _waiting = false;
        Thread.MemoryBarrier();
    }

    private static void Session_FrameReceived(object? sender, FrameReceivedEventArgs e)
    {
        switch ((STMessageCommand)e.Command)
        {
            case STMessageCommand.CmdRngResponse:
                {
                    if (STRngResponseMessage.TryRead(e.Data, out var rng, out var _))
                    {
                        Console.WriteLine("RNG Random Response:");
                        for (var i = 0; i < rng.Values.Length; ++i)
                        {
                            Console.WriteLine($"  {rng.Values[i]}");
                        }
                    }
                }
                break;
            case STMessageCommand.CmdGpioGetResponse:
                if (STGpioGetResponseMessage.TryRead(e.Data, out var gpioGet, out var _))
                {
                    Console.WriteLine("GPIO Get Response:");
                    var state = 0 == (gpioGet.Values & (1UL << _gpioNum)) ? "off" : "on";
                    Console.WriteLine($"  GPIO {_gpioNum} is {state}");
                }
                break;
            default:
                Console.WriteLine($"Unexpected frame {e.Command} received");
                break;
        }
        Console.WriteLine();
        _waiting = false;
        Thread.MemoryBarrier();
    }
}