import React from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";

export default function DailyInfo() {
    return (
        <div style={{ display: "flex", maxWidth: 900, marginTop: 20 }}>
            <Chart
                width={600}
                height={350}
                chartType="Calendar"
                loader={<CircularProgress />}
                data={[
                    [
                        { type: "date", id: "Date" },
                        { type: "number", id: "Won/Loss" },
                    ],
                    [new Date(2012, 3, 13), 37032],
                    [new Date(2012, 3, 14), 38024],
                    [new Date(2012, 3, 15), 38024],
                    [new Date(2012, 3, 16), 38108],
                    [new Date(2012, 3, 17), 38229],
                    [new Date(2013, 1, 4), 38177],
                    [new Date(2013, 1, 5), 38705],
                    [new Date(2013, 1, 12), 38210],
                    [new Date(2013, 1, 13), 38029],
                    [new Date(2013, 1, 19), 38823],
                    [new Date(2013, 1, 23), 38345],
                    [new Date(2013, 1, 24), 38436],
                    [new Date(2013, 2, 10), 38447],
                ]}
                options={{
                    title: "Daily Interactions",
                    calendar: { cellSize: 10 },
                }}
                rootProps={{ "data-testid": "1" }}
            />
        </div>
    );
}
