import React from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";

export default function ExhibitInfo() {
    return (
        <div style={{ display: "flex", maxWidth: 900 }}>
           <Chart
                width={800}
                height={"300px"}
                chartType="AreaChart"
                loader={<CircularProgress />}
                data={[
                    ["Date", "Exhibit 1", "Exhibit 2"],
                    ["3/23", 19, 23],
                    ["3/24", 21, 20],
                    ["3/25", 27, 21],
                    ["3/26", 16, 31],
                    ["3/27", 44, 33],
                    ["3/28", 0, 0],
                    ["3/29", 7, 12],
                    ["3/30", 21, 23],
                    ["3/31", 19, 56],
                    ["4/1", 21, 23],
                    ["4/2", 29, 19],
                    ["4/3", 36, 11],
                    ["4/4", 10, 25],
                ]}
                options={{
                    title: "Interactions throughout Exhibits",
                    hAxis: {
                        title: "Date - 2021",
                        titleTextStyle: { color: "#333" },
                    },
                    vAxis: { minValue: 0 },
                    // For the legend to fit, we make the chart area smaller
                    chartArea: { width: "50%", height: "70%" },
                    // lineWidth: 25
                }}
            />
        </div>
    );
}
