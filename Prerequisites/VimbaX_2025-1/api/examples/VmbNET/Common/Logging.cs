/*=============================================================================
  Copyright (C) 2024-2025 Allied Vision Technologies. All Rights Reserved.
  Subject to the BSD 3-Clause License.
=============================================================================*/

using Serilog.Configuration;
using Serilog.Core;
using Serilog.Events;
using Serilog;
using System.Diagnostics;
using System.Collections.Generic;
using Microsoft.Extensions.Logging;

namespace Logging
{
    /// <summary>
    /// Enrich log events with a "CallerInfo" property that describes where the event came from.
    /// </summary>
    class CallerInfoEnricher : ILogEventEnricher
    {
        public void Enrich(LogEvent logEvent, ILogEventPropertyFactory propertyFactory)
        {
            var method = new StackFrame(skip).GetMethod();

            bool isMethodInvalid() => method?.DeclaringType?.FullName == null
                                      || new List<string> { "Logger",
                                                            "CallerInfoEnricher",
                                                            "SafeAggregateEnricher", }.Find(item => method.DeclaringType.FullName.Contains(item)) != default(string);

            if (skip == 0 || isMethodInvalid())
            {
                skip = 0;
                bool found = false;
                while (!found)
                {
                    method = new StackFrame(++skip).GetMethod();
                    found = !isMethodInvalid();
                }
            }

            logEvent.AddPropertyIfAbsent(new LogEventProperty("CallerInfo",
                                                              new ScalarValue($"(in {method?.DeclaringType?.FullName}.{method?.Name})")));
        }

        static int skip = 0;
    }

    /// <summary>
    /// Add an extension method to <see cref="LoggerEnrichmentConfiguration"/> for including the <see cref="CallerInfoEnricher"/>.
    /// </summary>
    static class LoggerCallerInfoEnrichmentConfiguration
    {
        public static LoggerConfiguration WithCallerInfo(this LoggerEnrichmentConfiguration enrichmentConfiguration)
            => enrichmentConfiguration.With<CallerInfoEnricher>();
    }

    static class LoggerCreator
    {
        public static Microsoft.Extensions.Logging.ILogger CreateLogger()
        {
            Log.Logger = new LoggerConfiguration()
                .MinimumLevel.Debug()
                .Enrich.FromLogContext()
                .Enrich.WithCallerInfo()
//                .WriteTo.Console(outputTemplate: "[{Timestamp:yyyyMMdd HH:mm:ss.ffff} {Level:u3}] {Message} {CallerInfo}{NewLine}")
                .WriteTo.File("log.txt", outputTemplate: "[{Timestamp:yyyyMMdd HH:mm:ss.ffff} {Level:u3}] {Message} {CallerInfo}{NewLine}")
                .CreateLogger();
            ILoggerFactory factory = new LoggerFactory().AddSerilog(Log.Logger);
            return factory.CreateLogger("ConsoleApp");
        }
    }
}
