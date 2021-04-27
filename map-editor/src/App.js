import { useState, useEffect } from "react";
import { NavigationBar } from "./components/Navigation";
import Login from "./pages/Login";
import axios from "axios";
import { validateLink } from "./links";
import "./assets/styles/site.scss";

function App() {
    const [loggedIn, setLoggedIn] = useState(false);

    useEffect(() => {
        const token = localStorage.getItem("auth_token") || "";
        if (token === "") return;

        axios
            .get(validateLink, {
                headers: {
                    Accept: "application/json, text/plain, */*",
                    Authentification: token,
                    credentials: true,
                },
            })
            .then(({ data }) => {
                if (data.success) {
                    // console.log("Checked auth before with login state: " + loggedIn);
                    setLoggedIn(true);
                    // console.log("Checked auth after with login state: " + loggedIn);
                } else localStorage.removeItem("auth_token")
            });
    }, []);

    return (
        <div className="map-editor-body">
            {/* {console.log("Rerender app with login state: " + loggedIn)} */}
            {loggedIn ? (
                <NavigationBar />
            ) : (
                <div className="uwu">
                    <Login logIn={() => setLoggedIn(true)} />
                </div>
            )}
        </div>
    );
}

export default App;
