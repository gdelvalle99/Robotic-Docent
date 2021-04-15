import React, { useState, useEffect } from "react";
import Chart from "react-google-charts";
import { CircularProgress } from "@material-ui/core";
import { mapLink, floor_id } from "../links";

export default function RobotInfo() {
    const [image, setImage] = useState("");

    const getImage = () => {
        const formData = new FormData();
        formData.append("floor_id", floor_id); // just for testing

        const token = localStorage.getItem("auth_token") || "";
        fetch(mapLink, {
            method: "POST",
            headers: {
                Accept: "application/json, text/plain, */*",
                Authentification: token,
                credentials: true,
            },
            body: formData,
        })
            .then((response) => {
                return response.blob();
            })
            .then((blob) => {
                const link = URL.createObjectURL(blob);
                setImage(link);
            })
            .catch((e) => console.log(e));
    };
    useEffect(() => {
        getImage();
    }, []);

    return (
        <div style={{ display: "flex", maxWidth: 500, flexDirection: "column", alignItems: 'center', justifyContent: 'center' }}>
            <Chart
                width={400}
                height={"300px"}
                chartType="BubbleChart"
                loader={<CircularProgress />}
                data={[
                    [
                        "ID",
                        "Number of Interactions",
                        "Tour Duration",
                        "Tour Id",
                        "Group Size",
                    ],
                    ["3/25", 80.66, 1.67, "3/25", 3],
                    ["3/26", 79.84, 1.36, "3/26", 8],
                    ["3/27", 78.6, 1.84, "3/27", 2],
                    ["3/28", 72.73, 2.78, "3/28", 7],
                    ["3/29", 80.05, 2, "3/29", 6],
                    ["3/30", 72.49, 1.7, "3/30", 7],
                    ["3/31", 68.09, 4.77, "3/31", 3],
                    ["4/1", 81.55, 2.96, "4/1", 17],
                    ["4/2", 68.6, 1.54, "4/2", 14],
                    ["4/3", 78.09, 2.05, "4/3", 30],
                ]}
                options={{
                    title: "Duration of Tour vs Interactions",
                    hAxis: { title: "Number of Interactions" },
                    vAxis: { title: "Duration of Tour" },
                    bubble: { textStyle: { fontSize: 11 } },
                }}
            />
        </div>
    );
}
