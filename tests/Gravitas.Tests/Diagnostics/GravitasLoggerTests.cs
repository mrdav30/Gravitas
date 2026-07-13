using FluentAssertions;
using SwiftCollections.Diagnostics;
using System;
using System.Collections.Generic;
using System.IO;
using Xunit;

namespace Gravitas.Tests.Diagnostics;

public sealed class GravitasLoggerTests
{
    [Fact]
    public void DefaultFacade_ShouldFormatAndRouteNormalAndErrorOutput()
    {
        Func<DiagnosticLevel, string, string, string> originalFormatter = GravitasLogger.CustomFormatter;
        TextWriter originalOut = Console.Out;
        TextWriter originalError = Console.Error;
        using var output = new StringWriter();
        using var error = new StringWriter();

        string formatted;
        try
        {
            GravitasLogger.CustomFormatter = null!;
            Console.SetOut(output);
            Console.SetError(error);

            formatted = GravitasLogger.DefaultLogFormatter(DiagnosticLevel.Warning, "message", "Core");
            GravitasLogger.DefaultLogHandler(DiagnosticLevel.Info, "loaded", "Core");
            GravitasLogger.DefaultLogHandler(DiagnosticLevel.Error, "failed", "Core");
        }
        finally
        {
            Console.SetOut(originalOut);
            Console.SetError(originalError);
            GravitasLogger.CustomFormatter = originalFormatter;
        }

        formatted.Should().Be("[Warning] Gravitas.Core: message");
        output.ToString().Should().Be("[Info] Gravitas.Core: loaded" + Environment.NewLine);
        error.ToString().Should().Be("[Error] Gravitas.Core: failed" + Environment.NewLine);
    }

    [Fact]
    public void LoggerFacade_ShouldExposeConfiguredChannelsAndDebugGate()
    {
        bool originalDebugLogging = GravitasLogger.EnableDebugLogging;
        DiagnosticLevel originalMinimumLevel = GravitasLogger.MinimumLevel;
        Action<DiagnosticLevel, string, string> originalLogHandler = GravitasLogger.LogHandler;
        Func<DiagnosticLevel, string, string, string> originalFormatter = GravitasLogger.CustomFormatter;
        var entries = new List<(DiagnosticLevel Level, string Message, string Source)>();

        try
        {
            GravitasLogger.EnableDebugLogging = false;
            GravitasLogger.MinimumLevel = DiagnosticLevel.Info;
            GravitasLogger.LogHandler = (level, message, source) => entries.Add((level, message, source));
            GravitasLogger.CustomFormatter = (level, message, source) => $"{source}|{level}|{message}";

            GravitasLogger.Channel.Name.Should().Be("Gravitas");
            GravitasLogger.DebugChannel.Name.Should().Be("Gravitas");
            GravitasLogger.IsEnabled(DiagnosticLevel.Info).Should().BeTrue();
            GravitasLogger.CustomFormatter(DiagnosticLevel.Warning, "warned", "Core")
                .Should().Be("Core|Warning|warned");

            GravitasLogger.DebugChannel.Write(DiagnosticLevel.Info, "hidden", "Debug");
            GravitasLogger.Channel.Write(DiagnosticLevel.Info, "visible", "Core");
            GravitasLogger.EnableDebugLogging = true;
            GravitasLogger.DebugChannel.Write(DiagnosticLevel.Info, "debug", "Partition");
            GravitasLogger.MinimumLevel = DiagnosticLevel.Error;
            GravitasLogger.DebugChannel.Write(DiagnosticLevel.Warning, "filtered", "Partition");

            entries.Should().Equal(
                (DiagnosticLevel.Info, "visible", "Core"),
                (DiagnosticLevel.Info, "debug", "Partition"));

            GravitasLogger.LogHandler = null!;
            GravitasLogger.CustomFormatter = null!;

            GravitasLogger.LogHandler.Should().NotBeNull();
            GravitasLogger.CustomFormatter.Should().NotBeNull();
            GravitasLogger.CustomFormatter(DiagnosticLevel.Warning, "message", string.Empty)
                .Should().Be("[Warning] Gravitas: message");
            GravitasLogger.CustomFormatter(DiagnosticLevel.Warning, "message", "Core")
                .Should().Be("[Warning] Gravitas.Core: message");
        }
        finally
        {
            GravitasLogger.LogHandler = originalLogHandler;
            GravitasLogger.CustomFormatter = originalFormatter;
            GravitasLogger.MinimumLevel = originalMinimumLevel;
            GravitasLogger.EnableDebugLogging = originalDebugLogging;
        }
    }
}
