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
                    [new Date(2020, 3, 13), 132],
                    [new Date(2020, 3, 14), 124],
                    [new Date(2020, 3, 15), 124],
                    [new Date(2020, 3, 16), 108],
                    [new Date(2020, 3, 17), 229],
                    [new Date(2020, 1, 4), 377],
                    [new Date(2020, 1, 5), 305],
                    [new Date(2020, 1, 12), 210],
                    [new Date(2021, 1, 1), 0],
                    [new Date(2021, 1, 19), 323],
                    [new Date(2021, 1, 23), 345],
                    [new Date(2021, 1, 24), 236],
                    [new Date(2021, 2, 10), 347],
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
